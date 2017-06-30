/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 *  Updated on: Jun 25, 2017
 *      Author: Artem Artemev, @artemav
 */

#include "particle_filter.h"

#include <math.h>
#include <cassert>

#include <algorithm>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <iterator>


void ParticleFilter::init(double x, double y, double theta, double std[]) {
#if NDEBUG
  std::random_device rnd_dev;
  rnd_gen.seed(rnd_dev());
#else
  rnd_gen.seed(0);
#endif

  num_particles = 50;
  is_initialized = true;

  auto x_std = std[0];
  auto y_std = std[1];
  auto theta_std = std[2];

  norm_pdf x_pdf(x, x_std);
  norm_pdf y_pdf(y, y_std);
  norm_pdf theta_pdf(theta, theta_std);

  particles = std::vector<Particle>(num_particles);
  weights = std::vector<double>(num_particles);
  double weight = 1.0 / num_particles;
  std::fill(weights.begin(), weights.end(), weight);
  int id = 0;
  for (auto &particle : particles) {
      particle.id = id++;
      particle.weight = weight;
      particle.x = x_pdf(rnd_gen);
      particle.y = y_pdf(rnd_gen);
      particle.theta = theta_pdf(rnd_gen);
  }
}

void ParticleFilter::prediction(
    double dt,
    double std_pos[],
    double velocity,
    double yaw_rate) {
  auto x_std = std_pos[0];
  auto y_std = std_pos[1];
  auto theta_std = std_pos[2];

  norm_pdf x_pdf(0, x_std);
  norm_pdf y_pdf(0, y_std);
  norm_pdf theta_pdf(0, theta_std);

  yaw_rate = std::abs(yaw_rate) < epsilon ? epsilon : yaw_rate;
  for (auto &particle : particles) {
    auto x = particle.x;
    auto y = particle.y;
    auto theta = particle.theta;
    if (std::abs(theta) < epsilon) {
      x += velocity * dt * std::cos(theta);
      y += velocity * dt * std::sin(theta);
    } else {
      theta += yaw_rate * dt;
      x += velocity / yaw_rate * (std::sin(theta) - std::sin(particle.theta));
      y += velocity / yaw_rate * (std::cos(particle.theta) - std::cos(theta));
    }
    particle.x = x + x_pdf(rnd_gen);
    particle.y = y + y_pdf(rnd_gen);
    particle.theta = theta + theta_pdf(rnd_gen);
  }
}

void ParticleFilter::dataAssociation(
    const std::vector<LandmarkObs> &predicted,
    std::vector<LandmarkObs> *observations) {
  // NOTHING TO DO
}

void ParticleFilter::updateWeights(
    double sensor_range,
    double std_land[],
    const std::vector<LandmarkObs> &observations,
    const Map &map) {
  const auto x_var = std_land[0] * std_land[0];
  const auto y_var = std_land[1] * std_land[1];

  auto cov = std::sqrt(2 * M_PI * std_land[0] * std_land[1]);
  cov = std::max(cov, epsilon);
  int pow = num_particles / 2 + num_particles % 2;
  double weight_denom = std::max(std::pow(cov, pow), epsilon);

  double sum_weight = 0.0;
  for (auto &particle : particles) {
    auto cos = std::cos(particle.theta);
    auto sin = std::sin(particle.theta);
    auto x = particle.x;
    auto y = particle.y;
    auto cum_weight = 0.0;
    for (const auto &obs : observations) {
      auto xm = x + obs.x * cos - obs.y * sin;
      auto ym = y + obs.x * sin + obs.y * cos;
      Landmark landmark;
      if (!map.SearchNearestLandmark(xm, ym, &landmark)) {
        PF_DEBUG("Impossible happened - nearest points are not found");
        continue;
      }
      auto dx = landmark.x - xm;
      auto dy = landmark.y - ym;
      const auto weight = ((dx * dx) / x_var) + ((dy * dy) / y_var);
      cum_weight += weight;
    }
    particle.weight = std::exp(-0.5 * cum_weight) / weight_denom;
    sum_weight += particle.weight;
  }
  for (int i = 0; i < num_particles; ++i) {
    particles[i].weight /= sum_weight;
    weights[i] = particles[i].weight;
  }
}

void ParticleFilter::resample() {
  particles_pmf ppmf(weights.begin(), weights.end());
  std::vector<Particle> particles_next(num_particles);
  for (int i = 0; i < num_particles; ++i) {
    particles_next[i] = particles[ppmf(rnd_gen)];
  }
  particles = particles_next;
}

Particle ParticleFilter::SetAssociations(
    Particle particle,
    const std::vector<int> &associations,
    const std::vector<double> &sense_x,
    const std::vector<double> &sense_y) {
  particle.associations.clear();
  particle.sense_x.clear();
  particle.sense_y.clear();

  particle.associations= associations;
   particle.sense_x = sense_x;
   particle.sense_y = sense_y;

   return particle;
}

std::string ParticleFilter::getAssociations(const Particle &best) {
  std::vector<int> v = best.associations;
  std::stringstream ss;
    copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

std::string ParticleFilter::getSenseX(const Particle &best) {
  std::vector<double> v = best.sense_x;
  std::stringstream ss;
    std::copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}

std::string ParticleFilter::getSenseY(const Particle &best) {
  std::vector<double> v = best.sense_y;
  std::stringstream ss;
    std::copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
    std::string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
