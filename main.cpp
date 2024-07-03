/*
<header>

AUTHOR: Kyle T. Wylie
EST: <date>
*/

#include <SFML/Graphics.hpp>
#include <SFML/Audio.hpp>
#include <SFML/Window.hpp>
#include <SFML/System.hpp>

#include <array>
#include <iostream>
#include <string>
#include <cmath>
#include <vector>
#include <iomanip>
#include <functional>
#include <complex>
#include <thread>
#include <regex>
#include <mutex>

#include <X11/Xlib.h>   

#include "/home/kyle/Corporate/Programming/c++/ktw-lib/ktwgen.hpp"
#include "/home/kyle/Corporate/Programming/c++/ktw-lib/ktwmath.hpp"
#include "./polygon_rigidbody.cpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Pseudo-random number generator & initial seed. 
const int rng_seed = (uint64_t)time(0)+((uint64_t)time(0)<<32); 
ktw::llcaprng2 rng(rng_seed); 

std::mutex simutex; 

//Utility functions. 
#include "utils.hpp"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<rigidbody*> gons; 

size_t closest_to_mouse_index = 0; 
bool rmousedown = false; 
bool diagnostic = true; 
bool dograv = true; 
bool doattract = false; 

ktw::point collisionpoint = {0.0, 0.0}; 

std::vector<ktw::point> drawing_new_rigidbody; 

//Convert a body's temperature to a displayable colour (15 April '22). 
sf::Color temperature_to_colour(double temp, double midpoint_temp = 273.0) {
	double r = 255, b = 255; //Blue is to be "cold", red is to be "hot", white is to be "neutral" as defined by the midpoint. 
	return sf::Color(r, 255, b); 
}

/*
	Idealised physical soft body in two dimensions. 
	Capable of plastic deformation, and interaction with rigidbodies. 
	Based on project 'springsoftbodies' (20 Oct. '21). 

	AN1: WIP, basic premise right now is use vertices of polygon as pointmasses with edge springs, 
	...  and centerpoint mass with springs to vertices. Once complete put in ktwlib. 

	AUTHOR: Kyle T. Wylie
	EST: 15 April '22
*/
class softbody {
private: 
	//Internal pointmass structure
	struct pointmass {
		double m; //Mass of this body. 
		ktw::point x, dx; //Position and velocity. 
		//Methods. 
		void force(ktw::point f); //Accelerate (Euler integrate) this body according to Newton's second law. 
		void dampen(double d); //Multiply the velocity of this body by some factor, typically 0<d<1. 
		void tick(); //Advance this body one unit through time. 
	}; 
	//Internal Hooke's law abiding spring. 
	struct spring {
		pointmass *a, *b; //Pair of bodies linked by this spring. 
		double l0, k, d; //Rest length, spring constant, damping factor. 
		double l; //Current length. 
		//Methods. 
		double energy(); //Compute the potential energy stored in this spring. 
		void exert(); //Force the massive bodies to be a fixed distance apart. 
	}; 
	std::vector<pointmass> vertices; 
	std::vector<spring> springs; 
public: 
	softbody(polygon perimeter, double ivertex_mass, double ispring_constant); 
	//Force affecting. 
	//...
	//Physical interaction. 
	bool collide(rigidbody *rb, double damping = 1.0, ktw::point* collisionpoint = NULL); //Elastically collide with some other rigidbody, if applicable. 
	//Simulation. 
	void tick(); //Advance one simulation timestep. 
}; 

/*
	ktw::softbody
*/

void softbody::pointmass::force(ktw::point f) {
	dx = ktw::sum(dx, ktw::scale(1.0 / m, f)); 
}

void softbody::pointmass::dampen(double d) {
	dx = ktw::scale(d, dx); 
}

void softbody::pointmass::tick() {
	x = ktw::sum(x, dx); 
}

double softbody::spring::energy() {
	return 0.5 * k * pow(l - l0, 2.0); 
}

void softbody::spring::exert() {
	//Force the spring exerts (Hooke's Law). 
	l = ktw::distance(a->x, b->x); 
	double f = k * (l - l0); 
	//Handle damping. 
	ktw::point atob = ktw::hat(ktw::difference(b->x, a->x)); 
	ktw::point avtobv = ktw::difference(b->dx, a->dx); 
	double damp = d * ktw::dot(atob, avtobv); //How much the points are coming together / moving apart times the damping factor. 
	f += damp; //Incorporate damping. 
	atob = ktw::scale(f, atob); //Scale vector pointing between massive bodies by the force exerted. 
	//Apply force to bodies. 
	ktw::point btoa = ktw::hat(ktw::difference(a->x, b->x)); 
	btoa = ktw::scale(f, btoa); 
	a->force(atob); 
	b->force(btoa); 
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Setup, run once at start of program. 
void init() {
	/* AN: All bodies should have some initial rotation, otherwise perfectly axis aligned elements may not register. 
	gons.push_back(new rigidbody(polygon::regular_polygon(7, {width/2.0 - 100.0, height/2.0}, 60.0), 300.0, {0.25,0.0}, -0.01)); 
	gons.push_back(new rigidbody(polygon::regular_polygon(6, {width/2.0 - 50.0, height/2.0-10.0}, 45.0), 120.0, {0.25,0.0}, -0.01)); 
	gons.push_back(new rigidbody(polygon::regular_polygon(6, {width/2.0 + 50.0, height/2.0}, 55.0), 215.0, {0.15,0.0}, 0.002)); 
	gons.push_back(new rigidbody(polygon::regular_polygon(5, {width/2.0 + 100.0, height/2.0}, 50.0), 215.0, {0.15,0.0}, 0.002)); 
	gons.push_back(new rigidbody(polygon::regular_polygon(7, {width/2.0 - 100.0, height/2.0-30.0}, 60.0), 300.0, {0.25,0.0}, -0.01)); 
	gons.push_back(new rigidbody(polygon::regular_polygon(6, {width/2.0 - 50.0, height/2.0-40.0}, 45.0), 120.0, {0.25,0.0}, -0.01)); 
	gons.push_back(new rigidbody(polygon::regular_polygon(6, {width/2.0 + 50.0, height/2.0-30.0}, 55.0), 215.0, {0.15,0.0}, 0.002)); 
	gons.push_back(new rigidbody(polygon::regular_polygon(5, {width/2.0 + 100.0, height/2.0-30.0}, 50.0), 215.0, {0.15,0.0}, 0.002)); 
	//*/

	//Add the outer walls of the simulation. 
	//AN<12 April '22>: Having multiple walls decreases their "permeability" through simulation inaccuracies. 
	gons.push_back(new rigidbody(polygon::regular_polygon(4, {width/2.0 , height/2.0}, 650.0), std::numeric_limits<double>::max(), 273.0, {0.0,0.0}, 0.0)); 
	gons[gons.size() - 1]->rotate(0.0001 + ktw::pi / 4.0); 
	gons[gons.size() - 1]->transform(1.25, 0.0, 0.0, 1.0); 
	gons.push_back(new rigidbody(polygon::regular_polygon(4, {width/2.0 , height/2.0}, 651.0), std::numeric_limits<double>::max(), 273.0, {0.0,0.0}, 0.0)); 
	gons[gons.size() - 1]->rotate(0.0001 + ktw::pi / 4.0); 
	gons[gons.size() - 1]->transform(1.25, 0.0, 0.0, 1.0); 
	gons.push_back(new rigidbody(polygon::regular_polygon(4, {width/2.0 , height/2.0}, 652.0), std::numeric_limits<double>::max(), 273.0, {0.0,0.0}, 0.0)); 
	gons[gons.size() - 1]->rotate(0.0001 + ktw::pi / 4.0); 
	gons[gons.size() - 1]->transform(1.25, 0.0, 0.0, 1.0); 
}

//Perform these actions each tick. 
void tick(sf::RenderWindow* w) {
	simutex.lock(); 

	//Mouse forces. 
	if(rmousedown) {
		if(sf::Keyboard::isKeyPressed(sf::Keyboard::LShift)) {
			ktw::point tomouse = ktw::scale(1.0 * ktw::distance(gons[closest_to_mouse_index]->position(), mouse()), ktw::hat(ktw::from(gons[closest_to_mouse_index]->position(), mouse()))); 
			gons[closest_to_mouse_index]->force(tomouse); 
		} else {
			ktw::point tomouse = ktw::scale(0.5 * ktw::distance(gons[closest_to_mouse_index]->position(), mouse()), ktw::hat(ktw::from(gons[closest_to_mouse_index]->position(), mouse()))); 
			gons[closest_to_mouse_index]->force(tomouse); 
		}
	} else { //Find gon which is closes to cursor only if the mouse is not held. 
		double least_dist = std::numeric_limits<double>::max(), curr_dist; 
		closest_to_mouse_index = 0; 
		for(size_t i = closest_to_mouse_index; i < gons.size(); i++) {
			if(((curr_dist = ktw::distance(mouse(), gons[i]->position())) < least_dist) && (gons[i]->getmass() != 0.0) && (gons[i]->getmass() != std::numeric_limits<double>::max())) {
				least_dist = curr_dist; 
				closest_to_mouse_index = i; 
			}
		}
	}

	rigidbody::tick_all(gons, (dograv) ? 0.025 : 0.0, 0.3, 0.9999, 0.005, 0.25, 0.5, 5, &collisionpoint); 

	if(doattract) {
		for(size_t i = 0; i < gons.size(); i++) {
			for(size_t j = 0; j < gons.size(); j++) {
				if(i != j && !gons[i]->isstatic() && !gons[j]->isstatic()) {
					double af = 0.55 * gons[i]->getmass() * gons[j]->getmass() / pow(ktw::distance(gons[i]->position(), gons[j]->position()), 2.0); 
					gons[i]->force(ktw::scale(af, ktw::hat(ktw::from(gons[i]->position(), gons[j]->position())))); 
					gons[j]->force(ktw::scale(af, ktw::hat(ktw::from(gons[j]->position(), gons[i]->position())))); 
				}
			}
		}
	}

	simutex.unlock(); 
}

//Draw a single frame. 
void frame(sf::RenderWindow* w) {
	simutex.lock(); 

	//Draw gons. 
	for(size_t i = 0; i < gons.size(); i++) {
		sf::Color edgecolour = temperature_to_colour(gons[i]->gettemperature(), 0.0); 
		if(i == closest_to_mouse_index) {
			edgecolour = sf::Color::Cyan; 
		}
		if(diagnostic) draw_string(ktw::str(ktw::nearest(gons[i]->getmass(), 1.0)) + "kg", gons[i]->position().x, gons[i]->position().y, sf::Color::Green); 
		if(diagnostic) draw_string(ktw::str(ktw::nearest(gons[i]->gettemperature(), 1.0)) + "K", gons[i]->position().x, gons[i]->position().y + fontsize, sf::Color::Green); 
		if(diagnostic) draw_x(gons[i]->position().x, gons[i]->position().y, 5.0, sf::Color::Yellow); 
		for(size_t j = 0; j < gons[i]->size(); j++) {
			draw_line(gons[i]->side(j)[0].x, gons[i]->side(j)[0].y, gons[i]->side(j)[1].x, gons[i]->side(j)[1].y, edgecolour); 
			if(diagnostic) draw_cross(gons[i]->position().x + gons[i]->vertex(j).x, gons[i]->position().y + gons[i]->vertex(j).y, 5.0, sf::Color::Magenta); 

			/* (31 March '22) Correct production of vertex velocity vectors. 
			ktw::point pos = gons[i]->position(); //Position in absolute space. 
			ktw::point vert = gons[i]->vertex(j); //Position relative to body position. 

			ktw::point pos_plus_vert{pos.x + vert.x, pos.y + vert.y}; 
			ktw::point vert_linvel = gons[i]->getvelocity(); 
			ktw::point radial = ktw::from(pos, pos_plus_vert); 
			double vert_angspeed = ktw::norm(vert) * gons[i]->getangularvelocity(); 
			radial = ktw::hat(radial); 
			radial = {vert_angspeed * -radial.y, vert_angspeed * radial.x}; //Perpendicularise. 

			ktw::point vert_vel{radial.x + vert_linvel.x, radial.y + vert_linvel.y}; 

			draw_arrow(pos.x + vert.x, pos.y + vert.y, pos.x + vert.x + vert_vel.x, pos.y + vert.y + vert_vel.y, sf::Color::Green); 
			//*/
		}
		if(diagnostic) draw_x(collisionpoint.x, collisionpoint.y, 5.0, sf::Color::Green); 
	}

	//Draw points showing the gon being constructed, if applicable. 
	for(size_t i = 0; i < drawing_new_rigidbody.size(); i++) {
		draw_cross(drawing_new_rigidbody[i].x, drawing_new_rigidbody[i].y, 5.0, sf::Color::Red); 
	}

	simutex.unlock(); 
}

//Termination, run once at end of program. 
void cleanup() {
	for(size_t i = 0; i < gons.size(); i++) {
		delete gons[i]; 
		gons[i] = 0; 
	}
	gons.clear(); 
}

//Event handling function.
void eventhandle(sf::RenderWindow* w, sf::Event event) {
	bool shifting = sf::Keyboard::isKeyPressed(sf::Keyboard::LShift); 
	bool controling = sf::Keyboard::isKeyPressed(sf::Keyboard::LControl); 
	if(!shifting && drawing_new_rigidbody.size() > 0) { //When shift is released, create the new gon. 
		polygon tmp(drawing_new_rigidbody); 
		gons.push_back(new rigidbody(tmp, std::max(0.03 * tmp.area(), 50.0), 273.0 + rng.next<double>()*50.0, {0.0,0.0}, 0.00001)); 
		drawing_new_rigidbody.clear(); 
	}
	while(w->pollEvent(event)) {
		switch(event.type) {
			case sf::Event::Closed:
				cleanup(); 
				w->close();
				break;
			case sf::Event::KeyPressed:
				if(event.key.code == sf::Keyboard::R) {
					cleanup(); 
					init(); 
				} else if(event.key.code == sf::Keyboard::D) {
					diagnostic = !diagnostic; 
				} else if(event.key.code == sf::Keyboard::Z) {
					if(gons.size() > 2) gons.pop_back(); 
				} else if(event.key.code == sf::Keyboard::Delete) {
					if(gons.size() > 0) {
						delete gons[closest_to_mouse_index]; 
						gons.erase(gons.begin() + closest_to_mouse_index); 
					}
				} else if(event.key.code == sf::Keyboard::G) {
					dograv = !dograv; 
				} else if(event.key.code == sf::Keyboard::A) {
					doattract = !doattract; 
				}
				break; 
			case sf::Event::MouseButtonPressed:
				if(event.mouseButton.button == sf::Mouse::Left) {
					if(shifting && controling) { //Add a square. 
						polygon tmp =  polygon::regular_polygon(4, {mx(), my()}, 50.0); 
						tmp.rotate(0.00001 + ktw::pi/4.0); 
						gons.push_back(new rigidbody(tmp, 200.0, 273.0 + rng.next<double>()*50.0, {0.0,0.0}, 0.00001)); 
					} else if(shifting) { //Add a poing to the new gon list. 
						drawing_new_rigidbody.push_back(mouse()); 
					} else if(controling) { //Draw a "circle". 
						gons.push_back(new rigidbody(polygon::regular_polygon(30, {mx(), my()}, 30.0 + fabs(rng.next<double>()) * 40.0), 200.0 + fabs(rng.next<double>()) * 20.0, 273.0 + rng.next<double>()*50.0, {0.0,0.0}, 0.00001)); 
					} else {
						gons.push_back(new rigidbody(polygon::regular_polygon(3 + rng.next<unsigned>() % 6, {mx(), my()}, 30.0 + fabs(rng.next<double>()) * 40.0), 200.0 + fabs(rng.next<double>()) * 20.0, 273.0 + rng.next<double>()*50.0, {0.0,0.0}, 0.00001)); 
					}
				} else if(event.mouseButton.button == sf::Mouse::Right) {
					rmousedown = true; 
				}
				break; 
			case sf::Event::MouseButtonReleased: 
				if(event.mouseButton.button = sf::Mouse::Right) {
					rmousedown = false; 
				}
			default:
				break;
		}; 
	}
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//Graphical rendering thread.
void renderthread(sf::RenderWindow* w) {
	font.loadFromFile("tnr.ttf");
	text.setFont(font);
	text.setCharacterSize(fontsize);
	while(w->isOpen()) {
		w->clear(sf::Color::Black); //Clear. 
		//Draw all buttons. 
		for(size_t i = 0; i < buttons.size(); i++) buttons[i]->draw(w); 

		//Draw frame. 
		frame(w); 

		//Draw FPS. 
		draw_string(ktw::str((int) fps) + " fps, " + ktw::str((int) tps) + " tps", 10, 10, sf::Color::White, w); 
		//Initiate frame-draw. 
		w->display(); 
		//Wait some time per frame.
		ktw::wait(framedelay);
		frames_since_last++; 
		f++; 
	}
}

//Main program entry point.
int main() {
	XInitThreads(); //AN: Needed in Linux? 

	srand(time(NULL));
	sf::RenderWindow w(sf::VideoMode(width, height), "more like rockbox", sf::Style::Default);
	mw = &w; 
	w.setActive(false);

	init(); //Run any initial setup that must be done. 

	sf::Thread rt(&renderthread, &w);
	rt.launch();
	auto fps_t0 = ktw::timestamp();  //Timestamp for FPS. 
	while(w.isOpen()) {
		//Handle misc events.
		sf::Event event;
		eventhandle(&w, event);
		//Check if any buttons are being clicked. 
		for(size_t i = 0; i < buttons.size(); i++) buttons[i]->click(&w); 

		//Game events. 
		tick(&w); 

		//Calculate frames-per-second & ticks-per-second. 
		if(ktw::dur(fps_t0) >= fps_calc_delay) { //Every so often. 
			fps = (long double) frames_since_last / ktw::dur(fps_t0); //Compute FPS. 
			tps = (long double) ticks_since_last / ktw::dur(fps_t0); //Compute TPS. 
			frames_since_last = 0; //Reset frames-since-last check. 
			ticks_since_last = 0; 
			fps_t0 = ktw::timestamp(); //Reset timer. 
		}

		//Wait some time per tick. 
		ktw::wait(tickdelay); 
		ticks_since_last++; 
		t++; 
	}
	//Clean up and report normal exit. 
	for(size_t i = 0; i < buttons.size(); i++) delete buttons[i]; 
	return 0;
}
