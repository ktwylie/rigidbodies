/* (Applied)
	Implementation of an efficient 2-dimensional polygon. May function incorrectly if non-simple. 

	AN1: Put this into the proposed "ktwmath_applied" sublibrary of "ktwmath" which shall absorb certain functions from it. 
	...  Along with that, create a separate "ktwmath_pure" sublibrary for certain other functions from it. Which functions 
	...  go into which sublibrary and which remain in the primary header is to be determined more precisely later. 

	AN2<10 June '22>: If I'm being honest, this and rigidbody probably aren't generally useful enough to be in this library. 

	DEVNOTE1: Vertices "visible" to a point are those to which we can draw a ray without intersecting the polygon. 
	DEVNOTE2: Consider adding a variant/flag of each function to control getting values in absolute/relative space. 

	AUTHOR: Kyle T. Wylie
	EST: 18 June '21
*/
class polygon {
protected: 
	//Structural values. 
	ktw::point pos; //Origin of this polygon. 
	std::vector<ktw::point> pts; //Vertex coordinates. 
	//Precomputed values. 
	ktw::point bb1, bb2; //Corners of bounding box. 
	std::vector<std::vector<double>> normals; //Normal vectors of sides. 
	ktw::point com; //Center of mass point. 
	double plane_perimeter, plane_area; //Perimeter & area of this polygon. 
	std::vector<double> angles; //Angles of this polygon. 
	std::vector<double> lengths; //Side lengths. 
	//Precomputation utilities (piecemeal as most transformations don't change all of them). 
	void precompute_boundingbox(); //Precompute bounding box. 
	void precompute_normals(); //Precompute surface unit-normals. 
	void precompute_centerofmass(); //Precompute the center of mass. 
	void precompute_perimeter_area(); //Precompute the perimeter/area of this polygon via the Shoelase formula and sum of sidelengths. 
	void precompute_angles(); //Precompute the angles at each vertex via the law of cosines. 
	void precompute_sidelengths(); //Precompute the lengths of each side (must occur before precomputing perimeter). 
public: 
	//Constructors. 
	polygon(); //Empty constructor (produces a regular triangle at the origin). 
	polygon(ktw::point ipos, std::vector<ktw::point> ipts); //From origin and relative vertices. 
	polygon(std::vector<ktw::point> ipts); //From vertices in absolute space (origin shall be set to the vertex center-of-mass). 
	//Structural properties. 
	ktw::point position(); //Access origin position of this polygon. 
	size_t size(); //Number of vertices & sides of this polygon. 
	std::vector<ktw::point> side(size_t i); //Get the line segment for the ith side in absolute space. 
	ktw::point vertex(size_t i); //Get the ith vertex relative to origin. 
	//Derived properties. 
	ktw::point center_of_mass(); //Return the center of mass of this polygon. 
	double area(); //Return the area of this polygon. 
	double perimeter(); //Return the perimeter of this polygon. 
	std::vector<ktw::point> bounding_box(); //Get coordinates of bounding box in absolute space. 
	std::vector<double> normal(size_t i); //Get the normal vector for the ith side as a free vector. 
	double angle(size_t i); //Get the angle at vertex i. 
	double sidelength(size_t i); //Get the length of the ith side. 
	//Transformation. 
	void move(ktw::point dp); //Shift the origin of this polygon. 
	void set(ktw::point np); //Set the origin of this polygon. 
	void rotate(double dth); //Rotate this polygon about its origin. 
	void scale(double ds); //Scale this polygon relative to its origin. 
	void transform(double r1c1, double r1c2, double r2c1, double r2c2); //Apply a matrix to the vertices of this polygon. 
	//Interaction with other objects. 
	bool ray_intersection(ktw::point raysrc, ktw::point raydst, std::vector<ktw::point>* intersections = NULL, std::vector<size_t>* sides = NULL); //Does a vector intersect with some side? 
	bool polygon_intersection(polygon p, std::vector<ktw::point>* intersections = NULL); //Does this polygon intersect another? 
	bool point_within(ktw::point pt, ktw::point infinity = {NAN,NAN}); //Is a point inside the polygon's perimeter ("NAN,NAN" signals slower but universal approach)? 
	double distance_to(ktw::point pt); //Shortest Euclidean distance between 'pt' and the perimeter (negative if within). 
	std::vector<ktw::point> points_visible_from(ktw::point pt); //All the points on the polygon "visible" from 'pt'. 
	polygon intersection(polygon p); //Generate the polygon composed of the outermost perimeter of the intersection of these two (UNIMPLIMENTED: Compute intersection points, test vetrices for being within the polygon). 
	//Static methods. 
	static polygon regular_polygon(unsigned n, ktw::point origin, double scale = 1.0); //Generate a regular n-gon. 
	static polygon pythagorean_triple(unsigned m, unsigned n, unsigned k, ktw::point origin, double scale = 1.0); 
}; 

/*
	Idealised physical rigid body in two dimensions. 
	See N0<26-29 March '22> for some initial details regarding. 

	Mass of std::numeric_limits<double>::max() renders it fixed and interactible, zero is fixed and noninteractible. 

	AN1: Add feature such that objects whose velocity is below a certain threshold become "at rest" and don't move until a sufficiently 
	...  large change in their velocity occurs. Such objects can be cheaper computationally, and improve accuracy of simulation. 
	AN2: Further investigate proper heat equation dynamics to support temperature exchange. 
	AN3: Consider other properties that might compose such an object, for instance colour. 
	AN4: Should all the private variables simply be made public, and the getters/setters removed? 
	
	AUTHOR: Kyle T. Wylie
	EST: 27 March '22 (the third jewel perhaps?)
*/
class rigidbody : public polygon {
private: 
	double mass; //Mass of this body, distribution determined by COM/origin. 
	ktw::point2 velocity; //Change in position per timestep. 
	double angular_velocity; //Change in orientation per timestep. 
	double temperature; //Temperature of this rigidbody in Kelvin. 
	std::vector<rigidbody*> in_contact; //Bodies this one is currently in physical contact with. 
public: 
	rigidbody(ktw::point2 ipos, std::vector<ktw::point2> ipts, double imass, double itemp, ktw::point2 iv={0.0,0.0}, double iav=0.0); //Same as polygon constructor. 
	rigidbody(std::vector<ktw::point2> ipts, double imass, double itemp, ktw::point2 iv={0.0,0.0}, double iav=0.0); //Same as polygon constructor. 
	rigidbody(polygon p, double imass, double itemp, ktw::point2 iv={0.0,0.0}, double iav=0.0); //Construct a rigidbody from a polygon. 
	//Explicit attribute access. 
	double getmass(); //Yield the mass. 
	void setmass(double nmass); //Set the mass. 
	ktw::point2 getvelocity(); //Yield the velocity. 
	void setvelocity(ktw::point2 nvelocity); //Set the velocity. 
	double getangularvelocity(); //Yield the rate of rotation. 
	void setangularvelocity(double nangvel); //Set the rate of rotation. 
	double gettemperature(); //Yield the temperature. 
	void settemperature(double ntemperature); //Set the temperature. 
	//Implicit attribute access. 
	double kineticenergy(); //Compute the total kinetic energy of this body. 
	bool isstatic(); //Returns if this body is of a sort that it will be unaffected by certain interactions. 
	//Velocity affecting. 
	void dampen(double multiplier); //Scale velocities (usually 0 < multiplier < 1). 
	void accelerate(ktw::point2 acc_lin, double acc_ang); //Accelerate translationally and rotationally. 
	//Force affecting. 
	void force(ktw::point2 f); //Apply a strictly translational force. 
	void torque(ktw::point2 f, ktw::point2 r); //Apply a strictly rotational force at some radial distance. 
	void compositeforce(ktw::point2 f, ktw::point2 r); //Apply a force inducing possible translation/rotation. 
	//Physical interaction. 
	bool collide(rigidbody *rb, double damping = 1.0, ktw::point2* collisionpoint = NULL, size_t exclude_vertex_i = -1); //Elastically collide with some other rigidbody, if applicable. 
	static void deoverlap(std::vector<rigidbody*> gons, double shift_distance); //Perform a single deoverlap pass over a list of rigidbodies. 
	void thermal_conduct(rigidbody *rb, bool colliding, double conductivity); //Thermally conduct temperature ('colliding' reuses value from the 'collide' function). 
	void make_contact(rigidbody *rb); //Indicates that these bodies are now in physical contact. 
	//Simulation. 
	void tick(); //Apply velocities, advancing one simulation timestep. 
	static void tick_all(std::vector<rigidbody*> gons, double gravitation, double collision_damping, double air_damping, double conductivity, double deoverlap_setdist, double deoverlap_scale, unsigned deoverlap_iterations, ktw::point2 *collisionpoint); 
}; 

/*
	polygon
*/

/*
	ktw::polygon
*/

void ktw::polygon::precompute_boundingbox() {
	std::vector<ktw::point> bb = ktw::bounding_box(pts); 
	bb1 = bb[0]; 
	bb1.x += pos.x; //AN: Bounding box in absolute space, not relative to origin. 
	bb1.y += pos.y; 
	bb2 = bb[1]; 
	bb2.x += pos.x; 
	bb2.y += pos.y; 
}

void ktw::polygon::precompute_normals() {
	normals = {}; 
	std::vector<ktw::point> thisside; 
	for(size_t i = 0; i < size(); i++) { //AN: Surface normals are free vectors. 
		thisside = side(i); 
		normals.push_back(ktw::plane_normal(thisside[0], thisside[1], true)); 
	}
}

void ktw::polygon::precompute_centerofmass() {
	com = ktw::point{0.0, 0.0}; 
	ktw::point t; 
	for(size_t i = 0; i < pts.size(); i++) {
		t = pts[i]; 
		com.x += t.x; 
		com.y += t.y; 
	}
	com.x /= pts.size(); 
	com.y /= pts.size(); 
	com.x += pos.x; //AN: C.O.M. in absolute space, not relative to origin. 
	com.y += pos.y; 
}

void ktw::polygon::precompute_perimeter_area() {
	//Area. 
	plane_area = 0.0; 
	for(size_t i = 0; i < pts.size() - 1; i++) {
		plane_area += pts[i].x * pts[i+1].y; 
		plane_area -= pts[i+1].x * pts[i].y; 
	}
	plane_area += pts[pts.size() - 1].x * pts[0].y; 
	plane_area -= pts[0].x * pts[pts.size() - 1].y; 
	plane_area *= 0.5; 
	//Perimeter. 
	plane_perimeter = 0.0; 
	for(size_t i = 0; i < size(); i++) plane_perimeter += sidelength(i); 
}

void ktw::polygon::precompute_angles() {
	angles = {}; 
	double a, b, c; 
	for(size_t i = 0; i < pts.size(); i++) {
		size_t next = (i+1) % pts.size(), prev = (i == 0) ? pts.size()-1 : (i-1) % pts.size(); 
		b = ktw::distance(pts[i], pts[next]); 
		c = ktw::distance(pts[i], pts[prev]); 
		a = ktw::distance(pts[prev], pts[next]); 
		angles.push_back(acos((b*b + c*c - a*a) / (2.0*b*c))); 
	}
}

void ktw::polygon::precompute_sidelengths() {
	lengths = {}; 
	for(size_t i = 0; i < size(); i++) lengths.push_back(ktw::distance(side(i)[0], side(i)[1])); 
}

ktw::polygon::polygon() {
	*this = ktw::polygon::regular_polygon(3, {0.0, 0.0}, 1.0); 
}

ktw::polygon::polygon(ktw::point ipos, std::vector<ktw::point> ipts) { 
	pos = ipos; pts = ipts; 
	precompute_boundingbox(); 
	precompute_centerofmass(); 
	precompute_angles(); 
	precompute_normals(); 
	precompute_sidelengths(); 
	precompute_perimeter_area(); 
}

ktw::polygon::polygon(std::vector<ktw::point> ipts) {
	//Compute the average position of those points. 
	ktw::point com{0.0, 0.0}, t; 
	for(size_t i = 0; i < ipts.size(); i++) {
		t = ipts[i]; 
		com.x += t.x; 
		com.y += t.y; 
	}
	com.x /= ipts.size(); 
	com.y /= ipts.size(); 
	//Shift them all s.t. that point is the origin, construct a polygon as such. 
	for(size_t i = 0; i < ipts.size(); i++) {
		ipts[i].x -= com.x; 
		ipts[i].y -= com.y; 
	}
	*this = polygon(com, ipts); 
}

size_t ktw::polygon::size() {
	return pts.size(); 
}

std::vector<ktw::point> ktw::polygon::side(size_t i) {
	size_t index = i % size(), indexp1 = (i + 1) % size(); 
	ktw::point s1 = pts[index]; 
	s1.x += pos.x; //AN: Positions of side in absolute space. 
	s1.y += pos.y; 
	ktw::point s2 = pts[indexp1]; 
	s2.x += pos.x; 
	s2.y += pos.y; 
	return std::vector<ktw::point>{s1, s2}; 
}

void ktw::polygon::move(ktw::point dp) {
	pos.x += dp.x; 
	pos.y += dp.y; 
	precompute_boundingbox(); 
	precompute_centerofmass(); 
}

bool ktw::polygon::ray_intersection(ktw::point raysrc, ktw::point raydst, std::vector<ktw::point>* intersections, std::vector<size_t>* sides) {
	//Check if both raysrc and raydst of ray is out of bounding box. 
	if(!ktw::in_bounding_box(raysrc, bb1, bb2) && !ktw::in_bounding_box(raydst, bb1, bb2)) return false; 
	//At this point there is a possibility of intersection, check for it. 
	bool didsect = false; 
	for(size_t i = 0; i < size(); i++) {
		std::vector<ktw::point> thisside = side(i); 
		ktw::point intersection = ktw::segsect(raysrc, raydst, thisside[0], thisside[1]); 
		if(!std::isnan(intersection.x) && !std::isnan(intersection.y)) { //Intersection occured. 
			if(intersections != NULL) intersections->push_back(intersection); 
			if(sides != NULL) sides->push_back(i); 
			didsect = true; 
		}
	}
	return didsect; 
}

void ktw::polygon::rotate(double dth) {
	double r00 = cos(dth), r10 = -sin(dth); //Rotation matrix. 
	double r01 = -r10,     r11 = r00; 
	double tx, ty; 
	for(size_t i = 0; i < pts.size(); i++) { //Apply rotation matrix to all points. 
		tx = pts[i].x; 
		ty = pts[i].y; 
		pts[i].x = tx*r00 + ty*r10; 
		pts[i].y = tx*r01 + ty*r11; 
	}
	precompute_boundingbox(); 
	precompute_centerofmass(); //AN: May move if polygon is lopsided, since C.O.M. is not necessarily origin of rotation. 
	precompute_normals(); 
}

void ktw::polygon::scale(double ds) {
	for(size_t i = 0; i < pts.size(); i++) {
		pts[i].x *= ds; 
		pts[i].y *= ds; 
	}
	precompute_boundingbox(); 
	precompute_sidelengths(); 
	precompute_perimeter_area(); 
}

ktw::point ktw::polygon::center_of_mass() { return com; }

ktw::polygon ktw::polygon::regular_polygon(unsigned n, ktw::point origin, double scale) {
	double dth = ktw::tau / n; 
	std::vector<ktw::point> vertices; 
	for(double th = 0.0; th < ktw::tau; th += dth) vertices.push_back(ktw::point{scale*cos(th), scale*sin(th)}); 
	return polygon(origin, vertices); 
}

ktw::point ktw::polygon::position() { return pos; }

std::vector<double> ktw::polygon::normal(size_t i) { return normals[i % size()]; }

double ktw::polygon::area() { return plane_area; }

bool ktw::polygon::polygon_intersection(ktw::polygon p, std::vector<ktw::point>* intersections) {
	//Check the bounding boxes of us and them first. 
	std::vector<ktw::point> ourbb = bounding_box(), theirbb = p.bounding_box(); 
	ktw::point ourbb1 = ourbb[0], ourbb2 = ourbb[1], theirbb1 = theirbb[0], theirbb2 = theirbb[1]; 
	if(ourbb1.x > theirbb2.x || ourbb1.y > theirbb2.y || ourbb2.x < theirbb1.x || ourbb2.y < theirbb1.y) return false; 
	//At this point there is a chance that we are intersecting, look for one. 
	if(intersections != NULL) *intersections = {}; 
	bool didsect = false; 
	/* Code that reuses 'ray_intersection' and its bounding-box computation to save some resources. 
	for(size_t them_i = 0; them_i < p.size(); them_i++) {
		std::vector<ktw::point> theirside = p.side(them_i), anyintersections; 
		//AN: Need to order the coordinates of a side correctly via ktw::bounding_box, is that cheap enough to matter though? 
		didsect |= ray_intersection(theirside[0], theirside[1], &anyintersections); 
		if(intersections != NULL) intersections->insert(intersections->end(), anyintersections.begin(), anyintersections.end()); //AN: Only record if desired. 
	}
	//*/
	//* Keeping this code below in case the ordering on the ray-bounding-boxes turns out to be not worth it. 
	for(size_t us_i = 0; us_i < size(); us_i++) { //AN: Could also just iterate over all of their sides and do ray intersections with ourselves. 
		std::vector<ktw::point> ourside = side(us_i); 
		for(size_t them_i = 0; them_i < p.size(); them_i++) {
			std::vector<ktw::point> theirside = p.side(them_i); 
			//AN: At this point we could again do a bounding box check between segments, would need to order points with one closer to origin. 
			ktw::point intersection = ktw::segsect(ourside[0], ourside[1], theirside[0], theirside[1]); 
			if(!std::isnan(intersection.x) && !std::isnan(intersection.y)) {
				didsect = true; 
				if(intersections != NULL) intersections->push_back(intersection); //AN: Only record if desired to be so. 
			}
		}
	}
	//*/
	return didsect; 
}

std::vector<ktw::point> ktw::polygon::bounding_box() { return std::vector<ktw::point>{bb1, bb2}; }

void ktw::polygon::set(ktw::point np) {
	pos = np; 
	precompute_boundingbox(); 
	precompute_centerofmass(); 
}

bool ktw::polygon::point_within(ktw::point pt, ktw::point infinity) {
	if(!ktw::in_bounding_box(pt, bb1, bb2)) return false; //Check bounding box. 
	//bool didsect = false; 
	std::vector<ktw::point> intersections; 
	if(std::isnan(infinity.x) || std::isnan(infinity.y)) {
		//* Use 2x the distance from this point to the further point of the bounding box (faster). 
		double d1 = ktw::distance(pt, bb1), d2 = ktw::distance(pt, bb2), d = (d1 > d2) ? d1 : d2; 
		infinity = ktw::point{pt.x + 2.0*d, 0.0}; 
		ray_intersection(pt, infinity, &intersections); 
		//*/
		/* Use a line 2x the longest distance from the point to any vertex (slower). 
		double maxdist = ktw::distance(pt, pts[0]); 
		for(size_t i = 1; i < pts.size(); i++) {
			double dist = ktw::distance(pt, pts[i]); 
			if(dist > maxdist) maxdist = dist; 
		}
		ray_intersection(pt, ktw::point{pt.x + 2.0*maxdist, 0.0}, &intersections); 
		//*/
	} else { //Use user-specified infinity (faster). 
		ray_intersection(pt, infinity, &intersections); 
	}
	return intersections.size() % 2 != 0; //If the ray intersects an odd number of times, the point is within. 
}

double ktw::polygon::distance_to(ktw::point pt) {
	double mindist = 1.0 / 0.0; //AN: Okay since values will always be less than this. 
	double a, b, c, alpha, beta; 
	const double piover2 = ktw::pi / 2.0; 
	for(size_t i = 0; i < size(); i++) {
		std::vector<ktw::point> thisside = side(i); 
		a = ktw::distance(thisside[0], pt); 
		b = ktw::distance(thisside[1], pt); 
		c = sidelength(i); 
		alpha = acos((b*b + c*c - a*a) / (2.0*b*c)); 
		beta  = acos((a*a + c*c - b*b) / (2.0*a*c)); 
		bool alpha_ep = ktw::in(0.0, alpha, piover2), beta_ep = ktw::in(0.0, beta, piover2); 
		if(alpha_ep && beta_ep) { //If angle from this point to both endpoints is in 0 to 90 degrees, orth. dist. 
			double dist_to_edge = ktw::orthogonaldistance(pt, thisside[0], thisside[1]); 
			if(dist_to_edge < mindist) mindist = dist_to_edge; 
		} else if(alpha_ep) { //Otherwise, if in range from left-endpoint, nearer to right-endpoint. 
			double dist_alpha_ep = ktw::distance(pt, thisside[0]); 
			if(dist_alpha_ep < mindist) mindist = dist_alpha_ep; 
		} else if(beta_ep) { //...and vice versa. 
			double dist_beta_ep = ktw::distance(pt, thisside[1]); 
			if(dist_beta_ep < mindist) mindist = dist_beta_ep; 
		} else { //This should never happen. 
			std::cout << "THIS SHOULD NOT HAVE HAPPENED: SEE \'ktw::polygon::distance_to\' METHOD and investigate." << std::endl; 
			continue; 
		}
	}
	return (point_within(pt)) ? -mindist : mindist; 
}

ktw::point ktw::polygon::vertex(size_t i) { return pts[i % size()]; }

double ktw::polygon::angle(size_t i) { return angles[i % size()]; }

double ktw::polygon::sidelength(size_t i) { return lengths[i % size()]; }

double ktw::polygon::perimeter() { return plane_perimeter; }

ktw::polygon ktw::polygon::pythagorean_triple(unsigned m, unsigned n, unsigned k, ktw::point origin, double scale) {
	if(m <= n || m == 0 || n == 0) return ktw::polygon::regular_polygon(3, origin, scale); 
	double alen = scale * k * (m*m - n*n), blen = scale * k * 2*m*n; 
	ktw::point a = origin; 
	ktw::point b{origin.x + alen, origin.y}; 
	ktw::point c{origin.x + alen, origin.y + blen}; 
	return polygon(std::vector<ktw::point>{a, b, c}); 
}

void ktw::polygon::transform(double r1c1, double r1c2, double r2c1, double r2c2) {
	for(size_t i = 0; i < pts.size(); i++) {
		pts[i] = ktw::point{r1c1*pts[i].x + r1c2*pts[i].y, r2c1*pts[i].x + r2c2*pts[i].y}; 
	}
	//Need to recompute everything. 
	precompute_boundingbox(); 
	precompute_centerofmass(); 
	precompute_normals(); 
	precompute_sidelengths(); 
	precompute_angles(); 
	precompute_perimeter_area(); 
}

/*
	rigidbody
*/

rigidbody::rigidbody(ktw::point2 ipos, std::vector<ktw::point2> ipts, double imass, double itemp, ktw::point2 iv, double iav) : polygon(ipos, ipts) {
	mass = imass; 
	velocity = iv; 
	angular_velocity = iav; 
	temperature = itemp; 
}

rigidbody::rigidbody(std::vector<ktw::point2> ipts, double imass, double itemp, ktw::point2 iv, double iav) : polygon(ipts) {
	mass = imass; 
	velocity = iv; 
	angular_velocity = iav; 
	temperature = itemp; 
}

rigidbody::rigidbody(polygon p, double imass, double itemp, ktw::point2 iv, double iav) {
	//Set rigidbody parameters. 
	mass = imass; 
	velocity = iv; 
	angular_velocity = iav; 
	temperature = itemp; 
	//Extract defining information from the polygon. 
	pos = p.position(); 
	pts.clear(); 
	for(size_t i = 0; i < p.size(); i++) { //AN<29 March '22>: Unnecessary? 
		pts.push_back(p.vertex(i)); 
	}
	//Run precomputation. 
	precompute_boundingbox(); 
	precompute_centerofmass(); 
	precompute_angles(); 
	precompute_normals(); 
	precompute_sidelengths(); 
	precompute_perimeter_area(); 
}

void rigidbody::force(ktw::point2 f) {
	if(mass == 0.0) return; //Disregard massless objects. 
	//Apply force to this body. 
	velocity.x += f.x / mass; 
	velocity.y += f.y / mass; 
	//Apply normal force to contacting bodies. 
	//for(size_t i = 0; i < in_contact.size(); i++) in_contact[i]->force(f); 
}

void rigidbody::torque(ktw::point2 f, ktw::point2 r) { //AN: I can't quite shake the feeling that something's wrong here. 
	if(mass == 0.0) return; //Disregard massless objects. 
	//Apply force to this body. 
	double t = f.y*r.x - f.x*r.y; //Torque magnitude (AN: 2D cross product). 
	double i = mass * pow(ktw::norm(r), 2.0); //Moment of inertia. 
	angular_velocity += t / i; 
	//Apply normal force to contacting bodies. 
	//...
}

void rigidbody::compositeforce(ktw::point2 f, ktw::point2 r) {
	if(mass == 0.0) return; //Disregard massless objects. 
	//Radial vector pointing towards center. 
	ktw::point2 rr{-r.x, -r.y}; 
	//Compute component magnitudes. 
	double fmag = ktw::norm(f), rrmag = ktw::norm(rr); 
	double component = ktw::dot(f, rr) / (fmag * rrmag); //AN: Starts as cosine. 
	double lateral = fmag * component; 
	component = sin(acos(component)); //AN: Now sine. 
	double angular = fmag * component; 
	//Make rr a unit vector. 
	rr.x /= rrmag; 
	rr.y /= rrmag; 
	//Determine absolute angle of rr and f (so as to apply torque in proper direction). 
	double f_absang = atan2(f.y, f.x), rr_absang = atan2(rr.y, rr.x); 
	//Generate & apply component forces. 
	ktw::point2 radf{lateral * rr.x, lateral * rr.y}; 
	ktw::point2 tanf{angular * -rr.y, angular * rr.x}; //AN: Perpendicularise rr. 
	if(ktw::angle_less_than(f_absang, rr_absang)) {
		tanf.x = -tanf.x; 
		tanf.y = -tanf.y; 
	}
	force(radf); 
	torque(tanf, r); 
}

void rigidbody::tick() {
	//Reset list of contacting bodies (AN<CRITICAL>: Do not delete, just "forget"). 
	in_contact.clear(); 
	//Apply movement. 
	move(velocity); 
	rotate(angular_velocity); 
}

bool rigidbody::collide(rigidbody *rb, double damping, ktw::point2* collisionpoint, size_t exclude_vertex_i) { //29 March '22, to 12 April '22. 
	if(rb->getmass() == 0.0) return false; //Don't collide with massless objects. 

	//1: Iterate over vertices of this shape, using their next intended positions to determine if any collide with the other shape. 
	/*
		Currently only handles one vertex at a time, regardless of whether or not multiple would be colliding (an edge-on impact, for instance). 
		(22 April '22)
		Would it be sufficient if it simply checked for two vertex collisions (call itself with a new parameter "exclude_vertex_i")
		since theoretically no more than two should impact most of the time (edge-on collision). 
		I'm not sure how well this worked, although if I wished I could extend to 'collide with up to x vertices' by tracking
		a list of "don't collide again" vertices as well as a "qty_left_to_collide" parameter. 
	*/
	size_t colliding_vertex_i = -1; 
	std::vector<ktw::point2> rb_intersections; 
	std::vector<size_t> rb_sidesects; 
	ktw::point2 vertex_velocity, vertex_position_relative, vertex_position, vertex_position_next; 
	for(size_t i = 0; i < size(); i++) {
		if(i == exclude_vertex_i) continue; //Ignore the vertex collided with last time (22 April '22). 
		//Computation of the net velocity of this vertex. 
		ktw::point2 this_position = position(); 
		vertex_position_relative = vertex(i); 
		ktw::point2 orthogonal_at_vertex = ktw::hat(ktw::orthogonal(vertex_position_relative)); 
		double vertex_angular_speed = ktw::norm(vertex_position_relative) * getangularvelocity(); 
		vertex_velocity = ktw::sum(getvelocity(), ktw::scale(vertex_angular_speed, orthogonal_at_vertex)); 
		vertex_velocity = ktw::difference(vertex_velocity, rb->getvelocity()); //Consider velocity relative to other body. 
		//Vertex position this timestep and the next one. 
		vertex_position = ktw::sum(this_position, vertex_position_relative); 
		vertex_position_next = ktw::sum(vertex_position, vertex_velocity); 
		//Determine if this vertex will intersect with some edge of the other rigidbody. 
		if(rb->ray_intersection(vertex_position, vertex_position_next, &rb_intersections, &rb_sidesects)) {
			colliding_vertex_i = i; 
			break; 
		} else {
			rb_intersections.clear(); 
			rb_sidesects.clear(); 
		}
	}
	//Complete the initial collision detection pass. 
	if(colliding_vertex_i == -1) return false; //No collision occured. 
	vertex_velocity = ktw::sum(vertex_velocity, rb->getvelocity()); //Derelativise, was only to detect collisions better (edge moving into vertex). 

	//2: If one does, perform an elastic collision (w/ damping) on it and on the other object. 
	//Computation of the net velocity at the impact position. 
	ktw::point2 rb_position = rb->position(); 
	ktw::point2 rb_impact_position_relative = ktw::from(rb_position, rb_intersections[0]); //AN: Since intersection was in absolute space. 
	ktw::point2 rb_orthogonal_at_impact = ktw::hat(ktw::orthogonal(rb_impact_position_relative)); 
	double rb_impact_angular_speed = ktw::norm(rb_impact_position_relative) * rb->getangularvelocity(); 
	ktw::point2 rb_impact_velocity = ktw::sum(rb->getvelocity(), ktw::scale(rb_impact_angular_speed, rb_orthogonal_at_impact)); 
	//Impact position this timestep and the next one. 
	ktw::point2 rb_impact_position = rb_intersections[0]; //AN: Avoid recomputation. 
	ktw::point2 rb_impact_position_next = ktw::sum(rb_impact_position, rb_impact_velocity); 
	//Compute final velocities via the elastic collision equation. 
	double invmsum = mass + rb->getmass(); //Inverse of the sum of masses. 
	double v1_c1_coeff = (mass - rb->getmass()) / invmsum, v1_c2_coeff = (2*rb->getmass()) / invmsum; 
	double v2_c1_coeff = (2*mass) / invmsum, v2_c2_coeff = (rb->getmass() - mass) / invmsum; 
	ktw::point2 v1_c1 = ktw::scale(v1_c1_coeff, vertex_velocity), v1_c2 = ktw::scale(v1_c2_coeff, rb_impact_velocity); 
	ktw::point2 v2_c1 = ktw::scale(v2_c1_coeff, vertex_velocity), v2_c2 = ktw::scale(v2_c2_coeff, rb_impact_velocity); 
	ktw::point2 v1 = ktw::scale(damping, ktw::sum(v1_c1, v1_c2)); //New velocity of vertex on THIS body. 
	ktw::point2 v2 = ktw::scale(damping, ktw::sum(v2_c1, v2_c2)); //New velocity of vertex on OTHER body. 

	//3: Convert the change in velocity that results from the collision into a force and apply. 
	//Change in velocity is an acceleration, times a mass produces a force over a small time (impulse). 
	ktw::point2 f1 = ktw::scale(mass, ktw::difference(v1, vertex_velocity)); 
	ktw::point2 f2 = ktw::scale(rb->getmass(), ktw::difference(v2, rb_impact_velocity)); 

	/*
		Is it necessary to perform a reflection of the force vectors at this point? 
		Let's-a try. At least for f1 (the force exerted on the incident vertex). 
		ALSO: How do we get objects to push against each other (a continuous "impact" rather than a momentary one)? 
	*/
	//std::vector<double> f1_reflected{f1.x, f1.y}; 
	//f1_reflected = ktw::reflect(f1_reflected, rb->normal(rb_sidesects[0])); 
	//f1 = {f1_reflected[0], f1_reflected[1]}; 

	compositeforce(f1, vertex_position_relative); 
	rb->compositeforce(f2, rb_impact_position_relative); 

	//4: To handle fixed objects, perform a spatial shift to be no longer overlapping. 
	/* Outdated pairwise approach. Deprecated given the deoverlap static method. 
	if(getmass() <= rb->getmass()) { //Shift the body with the lower mass (arbitrary). 
		ktw::point2 shift = ktw::from(rb_impact_position, vertex_position); 
		move(shift); 
	} else {
		ktw::point2 shift = ktw::from(vertex_position, rb_impact_position); 
		rb->move(shift); 
	}
	//*/

	if(collisionpoint != NULL) *collisionpoint = rb_intersections[0]; //Record the location of the collision. 

	if(exclude_vertex_i == -1) collide(rb, damping, collisionpoint, colliding_vertex_i); //Collide up to one more time (22 April '22). 

	//in_contact.push_back(rb); //Add this body to the list of contacting bodies (AN<24 April '22>: Should this happen with deoverlapping?). 

	return true; //A collision has occured. 
}

double rigidbody::getmass() {
	return mass; 
}

void rigidbody::dampen(double multiplier) {
	angular_velocity *= multiplier; 
	velocity.x *= multiplier; 
	velocity.y *= multiplier; 
}

ktw::point2 rigidbody::getvelocity() {
	return velocity; 
}

double rigidbody::getangularvelocity() {
	return angular_velocity; 
}

double rigidbody::kineticenergy() {
	/* Can use this code once moment of inertia is computed. 
	double translational = 0.5 * mass * pow(ktw::norm(velocity), 2.0); 
	double rotational = 0.5 * moment_of_inertia * angular_velocity * angular_velocity; 
	return translational + rotational; 
	//*/
	//*
	return -1.0; //Placeholder, so that when I need it something will break and I'll fix it. 
	//*/
}

void rigidbody::deoverlap(std::vector<rigidbody*> gons, double shift_distance) {
	for(size_t i = 0; i < gons.size(); i++) {
		for(size_t j = 0; j < gons.size(); j++) {
			if(i == j) continue; //Don't deoverlap with self. 
			//1: Take the set of edgewise intersections of polygons...
			std::vector<ktw::point2> sects; 
			if(gons[i]->polygon_intersection(*gons[j], &sects)) {
				//1.5: Indicate that body i is in contact with body j. 
				//gons[i]->make_contact(gons[j]); //AN<24 April '22>: Seems to cause a hard crash. 
				//2: If any exist, compute the average position of all those points. 
				ktw::point2 average_intersection{0.0, 0.0}; 
				for(size_t k = 0; k < sects.size(); k++) average_intersection = ktw::sum(average_intersection, sects[k]); 
				average_intersection = ktw::scale(1.0 / sects.size(), average_intersection); 
				//3: Produce a unit vector from that average position to the centerpoint of the polygon to shift (preserve not shifting inf-mass). 
				//&4: Scale that vector to a desired "step distance" and shift the polygon along it. 
				/*
					AN PROPOSAL FOR IMPLEMENTING "AT REST" STATES: 
					1. Define an internal boolean value "at_rest". If this value is true, skip physics computations. 
					2. If a force is applied that would cause a velocity change to above the threshold, set at_rest to false. 
					3. Otherwise, if the velocity is currently below the threshold, set at_rest to true. 
					Something like this, details need to be worked out, but this would improve performance/accuracy greatly. 

					AN PROPOSAL FOR IMPLEMENTING CONTACT FORCE TRANSFER: 
					1. Each body keeps an internal list of other bodies it is in contact with, as well as the avg. contact location. 
					2. When a force/acceleration gets applied to a body, the component of that force in the direction of all
					   average contact locations gets applied to contacting bodies. 
					3. This list of contacting bodies gets updated by this method each tick. 
				*/
				//Shift both bodies equally, or the one without no/inf mass (13 April '22). 
				bool i_dontshift = gons[i]->getmass() == 0.0 || gons[i]->getmass() == std::numeric_limits<double>::max(); 
				bool j_dontshift = gons[j]->getmass() == 0.0 || gons[j]->getmass() == std::numeric_limits<double>::max(); 
				if(i_dontshift && j_dontshift) { //Shift neither. 
					continue; 
				} else if(i_dontshift) {
					//Shift j. 
					ktw::point2 avgint_to_j = ktw::from(average_intersection, gons[j]->position()); 
					ktw::point2 shifter = ktw::scale(shift_distance, ktw::hat(avgint_to_j)); 
					gons[j]->move(shifter); 
					//Cancel j linear velocity component in direction of shift. 
					ktw::point2 shifter_orth = {-shifter.y, shifter.x}; 
					ktw::point2 jvel = gons[j]->getvelocity(); 
					if(acos(ktw::dot(jvel, ktw::scale(-1.0, shifter))) < ktw::pi/4.0) { //Only cancel if moving towards the average intersection. 
						ktw::point2 jvel_cancel = ktw::proj(jvel, shifter_orth); 
						gons[j]->setvelocity(jvel_cancel); 
					}
				} else if(j_dontshift) {
					//Shift i. 
					ktw::point2 avgint_to_i = ktw::from(average_intersection, gons[i]->position()); 
					ktw::point2 shifter = ktw::scale(shift_distance, ktw::hat(avgint_to_i)); 
					gons[i]->move(shifter); 
					//Cancel i linear velocity component in direction of shift. 
					ktw::point2 shifter_orth = {-shifter.y, shifter.x}; 
					ktw::point2 ivel = gons[i]->getvelocity(); 
					if(acos(ktw::dot(ivel, ktw::scale(-1.0, shifter))) < ktw::pi/4.0) { //Only cancel if moving towards the average intersection. 
						ktw::point2 ivel_cancel = ktw::proj(ivel, shifter_orth); 
						gons[i]->setvelocity(ivel_cancel); 
					}
				} else {
					//Shift both. 
					ktw::point2 avgint_to_j = ktw::from(average_intersection, gons[j]->position()); 
					ktw::point2 avgint_to_i = ktw::from(average_intersection, gons[i]->position()); 
					ktw::point2 jshifter = ktw::scale(shift_distance, ktw::hat(avgint_to_j)); 
					ktw::point2 ishifter = ktw::scale(shift_distance, ktw::hat(avgint_to_i)); 
					gons[j]->move(jshifter); 
					gons[i]->move(ishifter); 
					//Cancel j linear velocity component in direction of shift. 
					ktw::point2 jshifter_orth = {-jshifter.y, jshifter.x}; 
					ktw::point2 jvel = gons[j]->getvelocity(); 
					ktw::point2 jvel_cancel; 
					if(acos(ktw::dot(jvel, ktw::scale(-1.0, jshifter))) < ktw::pi/4.0) { //Only cancel if moving towards the average intersection. 
						ktw::point2 jvel_cancel = ktw::proj(jvel, jshifter_orth); 
						gons[j]->setvelocity(jvel_cancel); 
						//gons[i]->accelerate(ktw::proj(jvel, jshifter), 0.0); //Transfer cancelled velocity component to other shape's velocity (17 April '22). 
					}
					//Cancel i linear velocity component in direction of shift. 
					ktw::point2 ishifter_orth = {-ishifter.y, ishifter.x}; 
					ktw::point2 ivel = gons[i]->getvelocity(); 
					if(acos(ktw::dot(ivel, ktw::scale(-1.0, ishifter))) < ktw::pi/4.0) { //Only cancel if moving towards the average intersection. 
						ktw::point2 ivel_cancel = ktw::proj(ivel, ishifter_orth); 
						gons[i]->setvelocity(ivel_cancel); 
						//gons[j]->accelerate(ktw::proj(ivel, ishifter), 0.0); //Transfer cancelled velocity component to other shape's velocity (17 April '22). 
					}
				}
				//*/
			}
		}
	}
}

void rigidbody::accelerate(ktw::point2 acc_lin, double acc_ang) {
	if(mass == std::numeric_limits<double>::max() || mass == 0.0) return; //Disregard fixed objects. 
	velocity = ktw::sum(velocity, acc_lin); 
	angular_velocity += acc_ang; 
}

void rigidbody::tick_all(std::vector<rigidbody*> gons, double gravitation, double collision_damping, double air_damping, double conductivity, double deoverlap_setdist, double deoverlap_scale, unsigned deoverlap_iterations, ktw::point2 *collisionpoint) {
	//Apply uniform gravitational force to all gons. 
	//for(size_t i = 0; i < gons.size(); i++) gons[i]->force({0.0, gravitation}); 

	//Apply uniform gravitational acceleration to all gons. 
	//AN<OUTDATED>: Not entirely sure why, but gravitation as a force seems to produce more "stable" collision results. 
	for(size_t i = 0; i < gons.size(); i++) gons[i]->accelerate({0.0, gravitation}, 0.0); 

	//Air damping. 
	for(size_t i = 0; i < gons.size(); i++) gons[i]->dampen(air_damping); 

	//Collisions and thermals. 
	for(size_t i = 0; i < gons.size(); i++) {
		for(size_t j = 0; j < gons.size(); j++) {
			bool did_collide; 
			if(i != j) {
				did_collide = gons[i]->collide(gons[j], collision_damping, collisionpoint); 
				gons[i]->thermal_conduct(gons[j], did_collide, conductivity); 
			}
		}
	}
	//Tick all gons. 
	for(size_t i = 0; i < gons.size(); i++) gons[i]->tick(); 

	//Perform multiple deoverlap passes. 
	for(size_t k = 0; k < deoverlap_iterations; k++) {
		rigidbody::deoverlap(gons, deoverlap_setdist); 
		deoverlap_setdist *= deoverlap_scale; 
	}
}

void rigidbody::setmass(double nmass) {
	mass = nmass; 
}

void rigidbody::setvelocity(ktw::point2 nvelocity) {
	velocity = nvelocity; 
}

void rigidbody::setangularvelocity(double nangvel) {
	angular_velocity = nangvel; 
}

double rigidbody::gettemperature() {
	return temperature; 
}

void rigidbody::settemperature(double ntemperature) {
	temperature = ntemperature; 
}

void rigidbody::thermal_conduct(rigidbody *rb, bool colliding, double conductivity) {
	if(!colliding) return; //If not in a state of contact, perform no thermal conduction. 
	//If in contact, allow heat to flow between the bodies. 
	double us_temp = gettemperature(), them_temp = rb->gettemperature(); 
	double diff = fabs(us_temp - them_temp); 
	if(us_temp < them_temp) { //AN<15 April '22>: Oversimplified "guess" model, look into heat equations (https://en.wikipedia.org/wiki/Thermal_contact_conductance). 
		if(!isstatic()) us_temp += conductivity * diff; 
		if(!rb->isstatic()) them_temp -= conductivity * diff; 
	} else {
		if(!isstatic()) us_temp -= conductivity * diff; 
		if(!rb->isstatic()) them_temp += conductivity * diff; 
	}
	settemperature(us_temp); 
	rb->settemperature(them_temp); 
}

bool rigidbody::isstatic() {
	return mass == 0.0 || mass == std::numeric_limits<double>::max(); 
}

void rigidbody::make_contact(rigidbody *rb) {
	in_contact.push_back(rb); 
}