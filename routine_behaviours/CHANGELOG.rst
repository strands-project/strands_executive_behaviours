^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package routine_behaviours
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.15 (2015-04-28)
-------------------
* increase force charge threshold to 30 to avoid battery level getting to low
* Contributors: Bruno Lacerda

0.0.14 (2015-04-21)
-------------------
* Removed redundant/confusing legacy code.
* Night tasks are now executed in a FIFO manner.
  Use this to fix strands-project/g4s_deployment/18
* Night tasks should now be executed in a FIFO order based on calls to add_night_task.
  This should be used to fix https://github.com/strands-project/g4s_deployment/issues/18
* Added ability to pull in tasks for just the day.
* Contributors: Nick Hawes

0.0.13 (2015-03-28)
-------------------
* Minor clean up.
* Add day off check to idle trigger.
  This fixes `#19 <https://github.com/strands-project/strands_executive_behaviours/issues/19>`_.
* Added titles for nicer deliverables
* added routine behaviour docs
* Resetting idle count after idle action
* Now doing a slightly smarter tour duration estimate
* Contributors: Nick Hawes

0.0.12 (2014-11-26)
-------------------
* Added simple follow task in case we want to try it later.
* Updated code to allow tasks at ChargingPoint when battery is low.
* Checking for is not None explicitly. Sorry @lucab-eyer.
* Merge pull request `#12 <https://github.com/strands-project/strands_executive_behaviours/issues/12>`_ from hawesie/hydro-devel
  Added rechecking for docking during the night.
* Use `is None` instead of `not`.
  Same as `strands-project/strands_executive#116 <https://github.com/strands-project/strands_executive/issues/116>`_
* Added rechecking for docking during the night.
* Contributors: Lucas Beyer, Nick Hawes

0.0.11 (2014-11-23)
-------------------
* Added db arg to method call.
* Contributors: Nick Hawes

0.0.10 (2014-11-23)
-------------------
* And added extra topic for @nilsbore
* Switched to colour topic for tweeting.
* Contributors: Nick Hawes

0.0.9 (2014-11-23)
------------------
* Added additional necessary replication.
* Added rgbd recording.
* Contributors: Nick Hawes

0.0.8 (2014-11-22)
------------------
* Added more flexible options for patrol behaviours.
* Contributors: Nick Hawes

0.0.7 (2014-11-21)
------------------
* Removed ChargingPoint from randomly visited nodes.
* Changed ptu sweep parameters
* Adding people_perception to the replication task
* Contributors: Christian Dondrup, Nick Hawes, Rares Ambrus

0.0.6 (2014-11-21)
------------------
* Expanded marathon routine.
* Added reasonably generic way of creating task routines.
* Contributors: Nick Hawes

0.0.5 (2014-11-18)
------------------
* Corrected day start and end
* Updated routinge to use new routine constraints.
* Cleaning up output and changing defaults for testing,
* Contributors: Nick Hawes

0.0.4 (2014-11-12)
------------------
* Fixed file permissions.
* Contributors: Nick Hawes

0.0.3 (2014-11-12)
------------------
* Corrected rosdep.
* Contributors: Nick Hawes

0.0.1 (2014-11-12)
------------------

* Updating CHANGELOGs.
* Moved patrol routine class into a src file.
* Basic patrolling routine for testing.
* Initial commit copied from strands_deployment.
* Contributors: Nick Hawes
