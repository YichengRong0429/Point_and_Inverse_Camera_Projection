# 3D_Reconstruction
The	goal	of	this	project	is	to implement	forward	(3D	point	to	2D	point)	and	inverse	(2D	point	to	
3D	 ray)	 camera	 projection,	 and	 to	 perform	 triangulation	 from	 two	 cameras	 to	 do	 3D	
reconstruction	from	pairs	of	matching	2D	image	points.		This	project	will	involve	understanding	
relationships	 between	 2D	 image	 coordinates	 and	 3D	 world	 coordinates	 and	 the	 chain	 of	
transformations	 that	 make	 up	 the	 pinhole	 camera	 model.

The specific	 tasks	 will	 be	 to	 project	 3D	 coordinates	 (sets	 of 3D	 joint	 locations	 on	 a	 human	 body,	
measured	by	motion	capture	equipment)	into	image	pixel	coordinates	that	I	can	overlay	on	
top	 of	 an	 image,	 to	 then	 convert	 those	 2D	 points	 back	 into	 3D	 viewing	 rays,	 and	 then	
triangulate	 the	 viewing	 rays	 of	 two camera	 views	 to	 recover	 the	 original	3D	 coordinates camera recorded	(or	values	close	to	those	coordinates)

