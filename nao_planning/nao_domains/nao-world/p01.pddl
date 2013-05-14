(define (problem p01)
	(:domain nao-world)
	(:objects
		dir-east - direction
		dir-north - direction
		dir-south - direction
	    	dir-northeast - direction
	    	dir-northwest - direction
	    	dir-southeast - direction
	    	dir-southwest - direction
	    	dir-west - direction
		bot - robot
		box-01 - box
		box-02 - box
		box-03 - box
		box-04 - box
		ball-01 - ball
		ball-02 - ball
		ball-03 - ball
		ball-04 - ball
		pos-01-01 - location
		pos-01-02 - location
		pos-01-03 - location
		pos-01-04 - location
		pos-02-01 - location
		pos-02-02 - location
		pos-02-03 - location
		pos-02-04 - location
		pos-03-01 - location
		pos-03-02 - location
		pos-03-03 - location
		pos-03-04 - location
		pos-04-01 - location
		pos-04-02 - location
		pos-04-03 - location
		pos-04-04 - location
	)
	(:init
		(HandEmpty bot)
		(at bot pos-01-01)
		(at box-01 pos-02-02)
		(at box-02 pos-04-02)
		(at box-03 pos-03-04)
		(at box-04 pos-04-03)
		(at ball-01 pos-01-03)
		(at ball-02 pos-03-03)
		(at ball-03 pos-01-02)
		(at ball-04 pos-02-01)
		(clear pos-01-04)
		(clear pos-02-03)
		(clear pos-02-04)
		(clear pos-03-01)
		(clear pos-03-02)
		(clear pos-04-01)
		(clear pos-04-04)
		(MOVE-DIR pos-01-01 pos-01-02 dir-east)	
		(MOVE-DIR pos-01-01 pos-02-02 dir-southeast)
		(MOVE-DIR pos-01-01 pos-02-01 dir-south)
		(MOVE-DIR pos-01-02 pos-01-01 dir-west)
		(MOVE-DIR pos-01-02 pos-02-01 dir-southwest)
		(MOVE-DIR pos-01-02 pos-02-02 dir-south)
		(MOVE-DIR pos-01-02 pos-02-03 dir-southeast)
		(MOVE-DIR pos-01-02 pos-01-03 dir-east)
		(MOVE-DIR pos-01-03 pos-01-02 dir-west)
		(MOVE-DIR pos-01-03 pos-02-02 dir-southwest)
		(MOVE-DIR pos-01-03 pos-02-03 dir-south)
		(MOVE-DIR pos-01-03 pos-02-04 dir-southeast)
		(MOVE-DIR pos-01-03 pos-01-04 dir-east)
		(MOVE-DIR pos-01-04 pos-01-03 dir-west)
		(MOVE-DIR pos-01-04 pos-02-03 dir-southwest)
		(MOVE-DIR pos-01-04 pos-02-04 dir-south)
		(MOVE-DIR pos-02-01 pos-01-01 dir-north)
		(MOVE-DIR pos-02-01 pos-03-01 dir-south)
		(MOVE-DIR pos-02-01 pos-03-02 dir-southeast)
		(MOVE-DIR pos-02-01 pos-02-02 dir-east)
		(MOVE-DIR pos-02-01 pos-01-02 dir-northeast)
		(MOVE-DIR pos-02-02 pos-01-02 dir-north)
		(MOVE-DIR pos-02-02 pos-01-01 dir-northwest)
		(MOVE-DIR pos-02-02 pos-02-01 dir-west)
		(MOVE-DIR pos-02-02 pos-03-01 dir-southwest)
		(MOVE-DIR pos-02-02 pos-03-02 dir-south)
		(MOVE-DIR pos-02-02 pos-03-03 dir-southeast)
		(MOVE-DIR pos-02-02 pos-02-03 dir-east)
		(MOVE-DIR pos-02-02 pos-01-03 dir-northeast)
		(MOVE-DIR pos-02-03 pos-01-03 dir-north)
		(MOVE-DIR pos-02-03 pos-01-02 dir-northwest)
		(MOVE-DIR pos-02-03 pos-02-02 dir-west)
		(MOVE-DIR pos-02-03 pos-03-02 dir-southwest)
		(MOVE-DIR pos-02-03 pos-03-03 dir-south)
		(MOVE-DIR pos-02-03 pos-03-04 dir-southeast)
		(MOVE-DIR pos-02-03 pos-02-04 dir-east)
		(MOVE-DIR pos-02-03 pos-01-04 dir-northeast)
		(MOVE-DIR pos-02-04 pos-01-04 dir-north)
		(MOVE-DIR pos-02-04 pos-01-03 dir-northwest)
		(MOVE-DIR pos-02-04 pos-02-03 dir-west)
		(MOVE-DIR pos-02-04 pos-03-03 dir-southwest)
		(MOVE-DIR pos-02-04 pos-03-04 dir-south)
		(MOVE-DIR pos-03-01 pos-02-01 dir-north)
		(MOVE-DIR pos-03-01 pos-04-01 dir-south)
		(MOVE-DIR pos-03-01 pos-04-02 dir-southeast)
		(MOVE-DIR pos-03-01 pos-03-02 dir-east)
		(MOVE-DIR pos-03-01 pos-02-02 dir-northeast)
		(MOVE-DIR pos-03-02 pos-02-02 dir-north)
		(MOVE-DIR pos-03-02 pos-02-01 dir-northwest)
		(MOVE-DIR pos-03-02 pos-03-01 dir-west)
		(MOVE-DIR pos-03-02 pos-04-01 dir-southwest)
		(MOVE-DIR pos-03-02 pos-04-02 dir-south)
		(MOVE-DIR pos-03-02 pos-04-03 dir-southeast)
		(MOVE-DIR pos-03-02 pos-03-03 dir-east)
		(MOVE-DIR pos-03-02 pos-02-03 dir-northeast)
		(MOVE-DIR pos-03-03 pos-02-03 dir-north)
		(MOVE-DIR pos-03-03 pos-02-02 dir-northwest)
		(MOVE-DIR pos-03-03 pos-03-02 dir-west)
		(MOVE-DIR pos-03-03 pos-04-02 dir-southwest)
		(MOVE-DIR pos-03-03 pos-04-03 dir-south)
		(MOVE-DIR pos-03-03 pos-04-04 dir-southeast)
		(MOVE-DIR pos-03-03 pos-03-04 dir-east)
		(MOVE-DIR pos-03-03 pos-02-04 dir-northeast)
		(MOVE-DIR pos-03-04 pos-02-04 dir-north)
		(MOVE-DIR pos-03-04 pos-02-03 dir-northwest)
		(MOVE-DIR pos-03-04 pos-03-03 dir-west)
		(MOVE-DIR pos-03-04 pos-04-03 dir-southwest)
		(MOVE-DIR pos-03-04 pos-04-04 dir-south)
		(MOVE-DIR pos-04-01 pos-03-01 dir-north)
		(MOVE-DIR pos-04-01 pos-04-02 dir-east)
		(MOVE-DIR pos-04-01 pos-03-02 dir-northeast)
		(MOVE-DIR pos-04-02 pos-03-02 dir-north)
		(MOVE-DIR pos-04-02 pos-03-01 dir-northwest)
		(MOVE-DIR pos-04-02 pos-04-01 dir-west)
		(MOVE-DIR pos-04-02 pos-04-03 dir-east)
		(MOVE-DIR pos-04-02 pos-03-03 dir-northeast)
		(MOVE-DIR pos-04-03 pos-03-03 dir-north)
		(MOVE-DIR pos-04-03 pos-03-02 dir-northwest)
		(MOVE-DIR pos-04-03 pos-04-02 dir-west)
		(MOVE-DIR pos-04-03 pos-04-04 dir-east)
		(MOVE-DIR pos-04-03 pos-03-04 dir-northeast)
		(MOVE-DIR pos-04-04 pos-03-04 dir-north)
		(MOVE-DIR pos-04-04 pos-03-03 dir-northwest)
		(MOVE-DIR pos-04-04 pos-04-03 dir-west)
	)
	(:goal 
		(at bot pos-04-04)
	)
	(:metric minimize (total-time))


)
