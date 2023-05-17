#include <fstream>
#include <functional>

#include <drake_vendor/tinyxml2.h>
#include <gflags/gflags.h>

#include "drake/multibody/parsing/parser.h"
#include "drake/multibody/tree/geometry_spatial_inertia.h"

DEFINE_bool(invalid_only, false,
            "if true, only fix physically invalid inertias");

namespace drake {
namespace multibody {
namespace {

using geometry::Role;
using geometry::SceneGraph;
using tinyxml2::XMLNode;
using tinyxml2::XMLDocument;
using tinyxml2::XMLElement;
using tinyxml2::XMLPrinter;

// XXX Does tinyxml2 quote-swapping ('' to "") break any strings?

// We jump through hoops here, just to get 2-space indented output.
class XmlPrinter final : public XMLPrinter {
 public:
  XmlPrinter() : XMLPrinter() {}
  void PrintSpace(int depth) final {
    for (int i = 0; i < depth; ++i) {
      Write("  ");
    }
  }
};

XMLElement* EnsureChildElement(XMLElement* el, const char* child) {
  XMLElement* kid = el->FirstChildElement(child);
  if (!kid) {
    kid = el->InsertNewChildElement(child);
  }
  return kid;
}

std::string roundtrip(double x) {
  return fmt::format("{}", x);
}

void FindLinks(XMLElement* el, std::vector<XMLElement*>* links) {
  if (std::string(el->Name()) == "link") {
    links->push_back(el);
    return;
  }
  for (XMLElement* kid = el->FirstChildElement();
       kid;
       kid = kid->NextSiblingElement()) {
    FindLinks(kid, links);
  }
}

class InertiaProcessor {
 public:
  InertiaProcessor(
      const MultibodyPlant<double>& plant,
      const SceneGraph<double>& scene_graph,
      XMLDocument* doc)
      : plant_(plant), scene_graph_(scene_graph), doc_(doc) {
    XMLElement* root = doc->RootElement();
    std::string root_name(root->Name());
    if (root_name == "sdf") {
      update_inertia_ =
          std::bind(&InertiaProcessor::UpdateInertiaSdf, this,
                    std::placeholders::_1, std::placeholders::_2);
    } else if (root_name == "robot") {
      update_inertia_ =
          std::bind(&InertiaProcessor::UpdateInertiaUrdf, this,
                    std::placeholders::_1, std::placeholders::_2);
    } else {
      drake::log()->error("Unknown file type: root element was {}", root_name);
      ::exit(EXIT_FAILURE);
    }

    // Crawl through the document, finding all of the 'link' elements,
    // regardless of their path back to the root. We can interrogate their
    // parents and deduce model names to ensure unique mappings.
    FindLinks(root, &links_);
  }

  std::optional<SpatialInertia<double>> MaybeFixBodyInertia(
      BodyIndex body_index) {
    // * Leverage the maybe-damaged model that got parsed by the parser:
    //   * for geometries
    //   * for mass/density parameters
    // * Build proper inertias from geometries:
    //   * cf. mujoco parser's techniques
    //   * figure out how to combine inertias for multiple geometries?
    const auto maybe_frame_id = plant_.GetBodyFrameIdIfExists(body_index);
    if (!maybe_frame_id.has_value()) {
      return {};
    }
    const auto& body_name = plant_.get_body(body_index).name();
    const auto& rigid_body = plant_.GetRigidBodyByName(body_name);
    const auto& old_inertia = rigid_body.default_spatial_inertia();
    if (FLAGS_invalid_only && old_inertia.IsPhysicallyValid()) {
      return {};
    }
    const double mass = old_inertia.get_mass();
    const auto& inspector = scene_graph_.model_inspector();
    const auto geoms = inspector.GetGeometries(*maybe_frame_id,
                                               Role::kProximity);
    if (geoms.empty()) {
      // XXX look at visuals instead??
      return {};
    }
    SpatialInertia<double> M_BBo_B(0, {0, 0, 0}, {0, 0, 0});
    for (const auto& geom : geoms) {
      const auto M_GG_G_one = CalcSpatialInertia(inspector.GetShape(geom), 1.0);
      SpatialInertia<double> M_GG_G(mass, M_GG_G_one.get_com(),
                                    M_GG_G_one.get_unit_inertia());
      const auto& X_BG = inspector.GetPoseInFrame(geom);
      const auto M_GBo_B = M_GG_G.ReExpress(X_BG.rotation())
                           .Shift(-X_BG.translation());
      // XXX presumably this accumulates CoM. How do we accumulate/understand
      // the inertia frame rotation?
      M_BBo_B += M_GBo_B;
    }
    return M_BBo_B;
  }

  void UpdateInertiaXml(
      BodyIndex body_index,
      const SpatialInertia<double>& inertia) {
    // * Edit fixed-up inertias back into doc.
    update_inertia_(body_index, inertia);
  }

 private:
  void UpdateInertiaUrdf(
      BodyIndex body_index,
      const SpatialInertia<double>& inertia) {
    const auto& body_name = plant_.get_body(body_index).name();
    XMLElement* found_link{};
    for (XMLElement* link : links_) {
      const char* name = link->Attribute("name");
      if (!name) { continue; }
      if (std::string(name) != body_name) { continue; }
      found_link = link;
      break;
    }
    DRAKE_DEMAND(found_link != nullptr);  // Named element was found.
    XMLElement* inertial = EnsureChildElement(found_link, "inertial");
    XMLElement* origin = EnsureChildElement(inertial, "origin");
    origin->SetAttribute("xyz", "0 0 0");
    origin->SetAttribute("rpy", "0 0 0");
    XMLElement* mass = EnsureChildElement(inertial, "mass");
    mass->SetAttribute("value", roundtrip(inertia.get_mass()).c_str());
    XMLElement* coeffs = EnsureChildElement(inertial, "inertia");
    const auto rot = inertia.CalcRotationalInertia();
    const auto mom = rot.get_moments();
    const auto prod = rot.get_products();
    coeffs->SetAttribute("ixx", roundtrip(mom(0)).c_str());
    coeffs->SetAttribute("iyy", roundtrip(mom(1)).c_str());
    coeffs->SetAttribute("izz", roundtrip(mom(2)).c_str());
    coeffs->SetAttribute("ixy", roundtrip(prod(0)).c_str());
    coeffs->SetAttribute("ixz", roundtrip(prod(1)).c_str());
    coeffs->SetAttribute("iyz", roundtrip(prod(2)).c_str());
  }

  void UpdateInertiaSdf(
      BodyIndex body_index,
      const SpatialInertia<double>& inertia) {
    // We may well be limited to a naive implementation of "write back to sdf"
    // here. Because sdf has nested models and (multiple kinds of?) inclusion,
    // it may be arbitrarily difficult to complete the job of writing back all
    // of the inertias in a multi-file model. It's probably possible to fix the
    // inertias in the primary document we loaded, but that's it.
    const auto& body_name = plant_.get_body(body_index).name();
    XMLElement* found_link{};
    for (XMLElement* link : links_) {
      // Find the one we want.
      const char* name = link->Attribute("name");
      if (!name) { continue; }
      if (std::string(name) != body_name) { continue; }
      // XXX use model instance names if needed. Maybe all this name-mapping
      // gets done up-front?
      found_link = link;
      break;
    }
    DRAKE_DEMAND(found_link != nullptr);
    // Fill out the inertial properties.
    XMLElement* inertial = EnsureChildElement(found_link, "inertial");
    XMLElement* pose = EnsureChildElement(inertial, "pose");
    pose->SetText("0 0 0 0 0 0");
    auto write_child = [&](XMLElement* parent, const char* name, double value) {
      XMLElement* el = EnsureChildElement(parent, name);
      el->SetText(roundtrip(value).c_str());
    };
    write_child(inertial, "mass", inertia.get_mass());
    XMLElement* coeffs = EnsureChildElement(inertial, "inertia");
    const auto rot = inertia.CalcRotationalInertia();
    const auto mom = rot.get_moments();
    const auto prod = rot.get_products();
    write_child(coeffs, "ixx", mom(0));
    write_child(coeffs, "iyy", mom(1));
    write_child(coeffs, "izz", mom(2));
    write_child(coeffs, "ixy", prod(0));
    write_child(coeffs, "ixz", prod(1));
    write_child(coeffs, "iyz", prod(2));
  }

  const MultibodyPlant<double>& plant_;
  const SceneGraph<double>& scene_graph_;
  XMLDocument* doc_{};
  std::vector<XMLElement*> links_;
  std::function<void(BodyIndex, const SpatialInertia<double>&)> update_inertia_;
};

int do_main(int argc, char* argv[]) {
  gflags::SetUsageMessage("[INPUT-FILE-OR-URL]\n"
                          "Rewrite URDF/SDFormat with fixed-up inertias");
  gflags::ParseCommandLineFlags(&argc, &argv, true);
  if (argc < 2) {
    drake::log()->error("missing input filename");
    return 1;
  }
  const char* outfile = (argc > 2) ? argv[2] : "/dev/stdout";

  XMLDocument xml_doc;
  xml_doc.LoadFile(argv[1]);
  if (xml_doc.ErrorID()) {
    drake::log()->error("Failed to parse XML file: {}",
                        xml_doc.ErrorName());
    ::exit(EXIT_FAILURE);
  }

  MultibodyPlant<double> plant{0.0};
  SceneGraph<double> scene_graph;
  plant.RegisterAsSourceForSceneGraph(&scene_graph);

  Parser parser{&plant};
  parser.package_map().PopulateFromRosPackagePath();

  parser.AddModels(argv[1]);

  InertiaProcessor processor(plant, scene_graph, &xml_doc);
  for (BodyIndex k(1); k < plant.num_bodies(); ++k) {
    const auto maybe_inertia = processor.MaybeFixBodyInertia(k);
    if (maybe_inertia.has_value()) {
      processor.UpdateInertiaXml(k, *maybe_inertia);
    }
  }

  // Use our custom XmlPrinter to "print to memory", then dump that to file.
  XmlPrinter printer;
  xml_doc.Print(&printer);
  std::ofstream out(outfile);
  out << printer.CStr();

  return EXIT_SUCCESS;
}

}  // namespace
}  // namespace multibody
}  // namespace drake

int main(int argc, char* argv[]) {
  return drake::multibody::do_main(argc, argv);
}