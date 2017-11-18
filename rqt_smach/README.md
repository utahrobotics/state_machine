These packages allow you to introspect the current state of the smach
state machine that is running.  The standard smach_viewer package on the ROS
wiki does not work; this a is [the workaround that someone posted](https://gist.github.com/matt3o/88bced95dba37a8932a51904d0734dff).


To run the introspector, build and: `rosrun rqt_smach rqt_smach`


Scrolling does not work for zooming, but ctrl+page[up|down] works.
