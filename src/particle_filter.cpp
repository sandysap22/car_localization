/*
 * particle_filter.cpp
 *
 *  
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>
#include <random>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  
  // cout << "in ParticleFilter::init start" << endl;
  
  num_particles=10;
  
  // random number generator 
  default_random_engine gen;
  // Create normal distributions for x, y and theta.
  normal_distribution<double> dist_x(x, std[0]);	
	normal_distribution<double> dist_y(y, std[1]);
	normal_distribution<double> dist_theta(theta, std[2]);
  
  double sample_x, sample_y, sample_theta;
  
  // cout << "Initital position " << x << " " << y << " " <<  theta << endl;
  // cout << "std " << std << endl;
  
  for(int i=0; i<num_particles; i++) {
    
    sample_x = dist_x(gen);
    sample_y = dist_y(gen);
    sample_theta = dist_theta(gen);
    
    Particle p;
    p.id=i;
    p.x=sample_x;
    p.y=sample_y;
    p.theta=sample_theta;
    p.weight=1.0;
    
    /**
    vector<int> associations;
    p.associations = associations;
    
    vector<double> sense_x;
    p.sense_x = sense_x;
    
    vector<double> sense_y;
    p.sense_y = sense_y;
    **/
    
    particles.push_back(p);
    
    weights.push_back(1.0);
  }
  
  
  is_initialized=true;

  // cout << "in ParticleFilter::init end" << endl;
  
}

void print_particles(vector<Particle> &particles){
   
  for(int i=0; i<particles.size(); i++) { 
    Particle p = particles[i];
    cout << p.id << ", " << p.x << ", " << p.y << ", " << p.theta << ", " << p.weight << endl;  
  }
  
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
  
  // cout << "in ParticleFilter::prediction start" << endl;
  
  default_random_engine gen;
  double THESH=0.001;
  
  for(int i=0; i<num_particles; i++) {  
  
    Particle p = particles[i];
    double old_x = p.x;
    double old_y = p.y;
    double old_yaw = p.theta;      

    double new_x,new_y,new_yaw=0.0;
    
    if (fabs(yaw_rate) >  THESH) {      
      new_x = old_x + velocity / yaw_rate * (sin(old_yaw+yaw_rate*delta_t) - sin(old_yaw));
      new_y = old_y + velocity / yaw_rate * (-cos(old_yaw+yaw_rate*delta_t) + cos(old_yaw));      
    } else {   
      new_x = old_x + velocity * delta_t * cos(old_yaw);
      new_y = old_y + velocity * delta_t * sin(old_yaw);      
    }
    
    new_yaw = old_yaw + yaw_rate * delta_t;
    
    normal_distribution<double> dist_x(new_x, std_pos[0]);
    normal_distribution<double> dist_y(new_y, std_pos[1]);
    normal_distribution<double> dist_theta(new_yaw, std_pos[2]);
    
    new_x = dist_x(gen);
    new_y = dist_y(gen);
    new_yaw = dist_theta(gen);    
    
    p.x = new_x;
    p.y = new_y;
    p.theta = new_yaw;
    
    particles[i] = p;
    
  }
  
  //cout << " particle position after update " << endl;
  //cout << " Vel and yaw rate was " << velocity << " " << yaw_rate << endl;
  // cout << "in ParticleFilter::prediction end" << endl;
}

Map::single_landmark_s findNearestNeighbor(double sensor_range,double pred_x,double pred_y,const Map &map_landmarks){
  
  // cout << "in findNearestNeighbor start" << endl;
  
  vector<Map::single_landmark_s> landmarks = map_landmarks.landmark_list;
  
  double dist_found = numeric_limits<double>::infinity();
  int map_id=-1;
  
  Map::single_landmark_s associatedLandmark;
  
  for(int k=0; k<landmarks.size(); k++){
      
    // now map it with nearest map landmark
    Map::single_landmark_s landmark = landmarks[k];

    // from helper class
    double euclidean_distance = dist(pred_x ,pred_y,landmark.x_f,landmark.y_f);
    
    //cout << "euclidean_distance " << euclidean_distance << endl;
    
    if(euclidean_distance < dist_found){
        dist_found = euclidean_distance;
        // map_id = k;
        associatedLandmark = landmark;
    }
  }
  
  // cout << "smallest dist " << dist_found <<  endl;
  //cout << pred_x << " : " << associatedLandmark.x_f << "  " << pred_y << " : " << associatedLandmark.y_f  <<  endl;
  // cout << "in findNearestNeighbor end" << endl;
  
  return associatedLandmark;
   
}


double getParticleWeight(double x_obs,double y_obs, Map::single_landmark_s associated_landmark,double std_landmark[]){
  
  // cout << "in getParticleWeight start" << endl;
  
  double sig_x = std_landmark[0];
  double sig_y = std_landmark[1];

  double mu_x = associated_landmark.x_f;
  double mu_y = associated_landmark.y_f;
  
  // calculate normalization term
  double gauss_norm= (1./(2. * M_PI * sig_x * sig_y));

  //cout << "gauss_norm " << gauss_norm << endl;  
  
  // calculate exponent
  double exponent= (pow((x_obs - mu_x),2))/(2 * pow(sig_x,2)) + (pow((y_obs - mu_y),2))/(2 *  pow(sig_y,2));
  
  //cout << "exponent " << exponent << endl;
  
  // calculate weight using normalization terms and exponent
  double  weight= gauss_norm * exp(-exponent);
  
  //cout << "weight " << weight << endl;
  
  // cout << "in getParticleWeight end" << endl;
  return weight;
  
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html
  
  // cout << "in ParticleFilter::updateWeights start" << endl;
 
  double xm,ym;
  int accociation_id;
  double particle_weight=1.0;
  
  // cout << "number of observations :" << observations.size() << endl;
  // cout << "transforming car coordinates to map coordinate" << endl;
  
  for(int i=0; i < particles.size(); i++)
  {
    Particle p = particles[i];
    double xp=p.x;
    double yp=p.y;
    double angle=p.theta;
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;
    particle_weight=1.0;
    Map::single_landmark_s associated_landmark;
    //vector<LandmarkObs> predictedLandmarks;
      
    // cout << "For particle " << p.id << endl;
    
    for (int j=0;j<observations.size();j++){
      
      LandmarkObs obs = observations[j];
      double x_from_car = obs.x;
      double y_from_car = obs.y;
      
      // cout << "obs " << x_from_car << "," << y_from_car << endl;
      
      // update observation in particle coordinate
      // do car coordinates transformation in map coordinates
      xm = xp + (cos(angle) * x_from_car) - (sin(angle) * y_from_car);
      ym = yp + (sin(angle) * x_from_car) + (cos(angle) * y_from_car);
      
      // cout << "transformed " << xm << "," << ym << endl;
      
      
      /**
      LandmarkObs predicted;
      predicted.x = xm;
      predicted.y = ym;      
      predictedLandmarks.push_back(predicted);
      */
     
      //int k = findNearestNeighbor(sensor_range,xm,ym,map_landmarks);
      associated_landmark = findNearestNeighbor(sensor_range,xm,ym,map_landmarks);
      
      /**
      if (k>0) {
         associated_landmark = map_landmarks[k];
      }else {
        continue;
      } */
      
      // calculate particle weight
      particle_weight *= getParticleWeight(xm,ym,associated_landmark,std_landmark);
      // do data accociation
      // then add each landmark id in association and coorespoinding xm as sensex and xy as sensey
      // populate list of association with landmarks and sensors
      associations.push_back(associated_landmark.id_i);
      sense_x.push_back(xm);
      sense_y.push_back(ym);  
      
    }
    // update final wight and associates 
    //cout << "going to set particle weight " << endl;
    particles[i].weight = particle_weight;
    // cout << "going to set associations " << endl;
    SetAssociations(p,associations,sense_x,sense_y);
    
    //cout << "going to set global weight : " << particle_weight << endl;
    // update global list
    weights[i]=particle_weight; 
    
  }  
  
  // cout << "in ParticleFilter::updateWeights end" << endl;
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
  
  
  //cout << "in ParticleFilter::resample start" << endl;
  
  
  random_device rd;
  mt19937 gen(rd());
  
  
  /**
  cout << "weights :" ;
  for(int i=0;i<weights.size();i++){
    
    cout << weights[i] << ",";
  }
   */
  
  //cout << endl;
  
  discrete_distribution<> d(weights.begin(),weights.end());
  
  /**
  cout << "weight probabilities " << endl;
  vector<double> p = d.probabilities();
  for(auto n : p)
    std::cout << n << ' ';
  std::cout << '\n';
  */
  
  // create copy of vector
  vector<Particle> old_particles(particles);
  
  particles.clear();
  //cout << "particle before sub sampling from " << endl;
  //print_particles(particles);
   
  for(int i=0; i < num_particles; i++)
  {
    int index = d(gen);
     
    Particle old_p = old_particles[index];  
    Particle new_p;
    
    // not sure about deep copy by simple assingment
    new_p.id = i;
    new_p.x = old_p.x;
    new_p.y = old_p.y;
    new_p.theta = old_p.theta;
    new_p.weight = old_p.weight;
    new_p.associations = old_p.associations;
    new_p.sense_x = old_p.sense_x;
    new_p.sense_y = old_p.sense_y;   
    
    particles.push_back(new_p);
    // particles[i] = new_p; // replace old particle
  }
  
  old_particles.clear();
  //cout << "particle after sub sampling from " << endl;
  //print_particles(particles);
  
  // cout << "in ParticleFilter::resample end" << endl;
}

void ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    // particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates
    // cout << "in ParticleFilter::SetAssociations start" << endl;
    
    // cout << "setting associates" << endl;
    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
    
    // cout << "in ParticleFilter::SetAssociations end" << endl;

}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
