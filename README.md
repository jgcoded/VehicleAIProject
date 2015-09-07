# VehicleAIProject
Custom path following and AI for vehicles in Unreal Engine.

![](http://i.imgur.com/vCHaLjK.gif)

Completed:

* Vehicles can move to a target using the MoveTo functions

Issues:

* Fix up the math used with the PIDControllers in the VehiclePathFollowingComponent::FollowPathSegment()
* Calculate throttle and steering inputs a better way
  * Throttle is calculated based on the percentage of distance left to the destination. This is horrible from a math point of view.
  * Steering is calculated based on the angle to the destination. This is ok, but it can be made more correct.
* Sometimes the vehicle gets stuck on going forward/reverse.
* Sometimes the vehicle gets stuck on edges of objects.
* Sometimes the generated path assumes the vehicle can step onto a rising slope.
