======================
 Simple Car Explained
======================

Requirements
============

 * model *kinematics* of a car, in (somewhat) idiomatic Drake

   * eventual goal is to scale to a large number of "non-player" cars

 * manual driving input (for first iteration)
 * visualize the result in drake-visualizer, neglecting almost all details

Requirements
============

 * provide (excessive?) logging to support debugging
 * model some typical best practices

   * unit tests
   * DRY -- Don't Repeat Yourself
   * automate rather than document (sorry, Windows ;) )

Design
======

 * car kinematics

   * simplest thing that could possibly work
   * output just exposes state, for later visualization

 * driving input

   * just a cleaned-up version of the input app from Cars/

 * visualization

   * simplest possible
   * just a block that slides around

Design
======

 * unit tests

   * kinematics only
   * test properties rather than exact values

 * logging

   * LCM everywhere

 * automation

   * simple bash script to run all components reliably

Surprises
=========

 * dead-simple visualization requires mysterious URDF hacks

Bonuses
=======

 * LCM logging helpers

   * but, LCM Vector concept is tedious, so lcm_vector_gen.py
   * extract LCMTap from some LCMSystem internals

 * unit test

   * contains some possibly-useful System concept test helpers

Complaints
==========

 * autodiff? -- not an initial goal, but probably still achievable
 * bash scripts? -- could be ported to python for Windows compatibility
 * more documentation? -- guilty as charged

Conclusions
===========

 * starting point for "moving scenery" to use with sensor modeling and traffic
 * demonstrate non-physical modeling within Drake
 * follow-on systems will likely different:

   * scriptable
   * ability to scale to many vehicles

.. class:: handout

           The point of SimpleCar was not so much to study a dynamical system
           of direct interest, but rather to create "moving scenery" that could
           be the basis of sensor modeling and traffic interaction.

           Having the ability to do non-physical modeling inside Drake will be
           useful for building out complex scenarios, and helps to deflect the
           criticism of Drake that it intends to "build the Matrix".

           SimpleCar will not likely be used at scale in its present state, but
           morphed into Nimmer's distance-parametric system for trajectory
           following, which is much more scriptable.
