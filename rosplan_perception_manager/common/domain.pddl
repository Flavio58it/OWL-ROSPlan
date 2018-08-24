(define
	(domain robot_explore)
	(:requirements :strips :typing  :disjunctive-preconditions :negative-preconditions)

	(:types
		waypoint
		robot
		object
	)
	(:predicates
		(robot_at ?v - robot ?wp - waypoint)  ; true iff the robot is at the waypoint
		(connected ?from ?to - waypoint)
		(visible ?v - robot ?wp - waypoint ?obj - object)
		(detected ?obj - object)
	)

	;; Use perception actions to search for objects at the current waypoint
	(:action grasp
		:parameters (?v - robot ?wp - waypoint ?obj - object)
		:precondition (and
			(robot_at ?v ?wp)
			(visible ?v ?wp ?obj)
		)
		:effect (and
					(detected ?obj)
					(not (visible ?v ?wp ?obj))
		)
	)

	;; Move between any two waypoints
	(:action move
		:parameters (?v - robot ?from ?to - waypoint)
		:precondition (and
			(robot_at ?v ?from)
			(connected ?from ?to)
			)
		:effect (and
			(not (robot_at ?v ?from))
			(robot_at ?v ?to)
		)
	)
)
