#include "3rd/line_extraction/line.h"

namespace dslam
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
Line::Line(const CachedData &c_data, const RangeData &r_data, const Params &params, 
           std::vector<unsigned int> indices):
  c_data_(c_data),
  r_data_(r_data),
  params_(params),
  indices_(indices)
{
}

Line::Line(double angle, double radius, const boost::array<double, 4> &covariance,
       const boost::array<double, 2> &start, const boost::array<double, 2> &end,
       const std::vector<unsigned int> &indices):
  angle_(angle),
  radius_(radius),
  covariance_(covariance),
  start_(start),
  end_(end),
  indices_(indices)
{
}

Line::Line(boost::array<double, 2> start,boost::array<double, 2> end):
  start_(start),
  end_(end)
  {
    // TODO calculate other variable here
  }

Line::~Line()
{
}

///////////////////////////////////////////////////////////////////////////////
// Get methods for line parameters
///////////////////////////////////////////////////////////////////////////////
double Line::getAngle() const
{
  return angle_;
}

const boost::array<double, 4>& Line::getCovariance() const
{
  return covariance_;
}

const boost::array<double, 2>& Line::getEnd() const
{
  return end_;
}
const boost::array<double, 2>  Line::getCenter() const
{
  boost::array<double, 2> point;
  point[0] = (start_[0] + end_[0])/2.0;
  point[1] = (start_[1] + end_[1])/2.0;
  return point;
}
const boost::array<double, 2>  Line::getPerpendicular() const {
  boost::array<double, 2> point;
  // first convert line to normalized unit vector
  double dx = end_[0] - start_[0];
  double dy = end_[1] - start_[1];
  double mag = sqrt(dx*dx + dy*dy);
  dx /= mag;
  dy /= mag;

  // translate the point and get the dot product
  double lambda = (dx * (- start_[0])) + (dy * (-start_[1]));
  point[0] = (dx * lambda) + start_[0];
  point[1] = (dy * lambda) + start_[1];
  return point;
}

const double Line::dist() const
{
  boost::array<double, 2> point = getPerpendicular();
  return sqrt(pow(point[0],2) + pow(point[1],2));
}

const std::vector<unsigned int>& Line::getIndices() const
{
  return indices_;
}

double Line::getRadius() const
{
  return radius_;
}

const boost::array<double, 2>& Line::getStart() const
{
  return start_;
}
void Line::asPointCloud(std::vector<pcl::PointXYZ>& cloud, tf::Transform& transform, int size, double lim) const
{
  tf::Vector3 nend, nstart;
  nend.setX(end_[0]);
  nend.setY(end_[1]);
  nstart.setX(start_[0]);
  nstart.setY(start_[1]);
  nend = transform*nend;
  nstart = transform*nstart;

  //nend.setX(nend.x() - transform.getOrigin().getX());
  //nend.setY(nend.y() - transform.getOrigin().getY());

  //nstart.setX(nstart.x() - transform.getOrigin().getX());
  //nstart.setY(nstart.y() - transform.getOrigin().getY());

  double dx = nend.x() - nstart.x();
  double dy = nend.y() - nstart.y();
  double mag = sqrt(dx*dx + dy*dy);
  dx /= mag;
  dy /= mag;
  //int size = r_data_.xs.size()/scale;
  boost::array<double, 2> a,b;
  a[0] = 0.0;
  a[1] = 0.0;
  pcl::PointXYZ point;
  for(int i = 0; i < size ; i++)
  {
    point.x = b[0] = dx*i*mag/(double)size + nstart.x();
    point.y = b[1] = dy*i*mag/(double)size + nstart.y();
    point.z = 0.0;
    Line l(a,b);
    if(l.length() > lim) continue;
    cloud.push_back(point);

  }
  point.x = nstart.x();
  point.y = nstart.y();
  cloud.push_back(point);

  point.x = nend.x();
  point.y = nend.y();
  cloud.push_back(point);
  /*cloud.resize(3);
  tf::Vector3 landmark;
  landmark.setX(end_[0]);
  landmark.setY(end_[1]);
  landmark = transform*landmark;
  cloud[0].x = landmark.x();
  cloud[0].y = landmark.y();

  landmark.setX(start_[0]);
  landmark.setY(start_[1]);
  landmark = transform*landmark;
  cloud[1].x = landmark.x();
  cloud[1].y = landmark.y();

  landmark.setX(getPerpendicular()[0]);
  landmark.setY(getPerpendicular()[1]);
  landmark = transform*landmark;
  cloud[2].x = landmark.x();
  cloud[2].y = landmark.y();*/
}
///////////////////////////////////////////////////////////////////////////////
// Utility methods
///////////////////////////////////////////////////////////////////////////////
double Line::distToPoint(unsigned int index)
{
  double p_rad = sqrt(pow(r_data_.xs[index], 2) + pow(r_data_.ys[index], 2));
  double p_ang = atan2(r_data_.ys[index], r_data_.xs[index]);
  return fabs(p_rad * cos(p_ang - angle_) - radius_);
}

double Line::length() const
{
  return sqrt(pow(start_[0] - end_[0], 2) + pow(start_[1] - end_[1], 2));
}

unsigned int Line::numPoints() const
{
  return indices_.size();  
}

void Line::projectEndpoints()
{
  double s = -1.0 / tan(angle_);
  double b = radius_ / sin(angle_);
  double x = start_[0];
  double y = start_[1];
  start_[0] = (s * y + x - s * b) / (pow(s, 2) + 1);
  start_[1] = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
  x = end_[0];
  y = end_[1];
  end_[0] = (s * y + x - s * b) / (pow(s, 2) + 1);
  end_[1] = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
}

///////////////////////////////////////////////////////////////////////////////
// Methods for endpoint line fitting
///////////////////////////////////////////////////////////////////////////////
void Line::endpointFit()
{
  start_[0] = r_data_.xs[indices_[0]]; 
  start_[1] = r_data_.ys[indices_[0]]; 
  end_[0] = r_data_.xs[indices_.back()]; 
  end_[1] = r_data_.ys[indices_.back()]; 
  angleFromEndpoints();
  radiusFromEndpoints();
}

void Line::angleFromEndpoints()
{
  double slope;
  if (fabs(end_[0] - start_[0]) > 1e-9)
  {
    slope = (end_[1] - start_[1]) / (end_[0] - start_[0]);
    angle_ = pi_to_pi(atan(slope) + M_PI/2);
  }
  else
  {
    angle_ = 0.0;
  }
}

void Line::radiusFromEndpoints()
{
  radius_ = start_[0] * cos(angle_) + start_[1] * sin(angle_);
  if (radius_ < 0)
  {
    radius_ = -radius_;
    angle_ = pi_to_pi(angle_ + M_PI);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Methods for least squares line fitting
///////////////////////////////////////////////////////////////////////////////
void Line::leastSqFit()
{
  calcPointCovariances();
  double prev_radius = 0.0;
  double prev_angle = 0.0;
  while (fabs(radius_ - prev_radius) > params_.least_sq_radius_thresh ||
         fabs(angle_ - prev_angle) > params_.least_sq_angle_thresh) 
  {
    prev_radius = radius_;
    prev_angle = angle_;
    calcPointScalarCovariances();
    radiusFromLeastSq();
    angleFromLeastSq();
  }
  calcCovariance();
  projectEndpoints();
}

void Line::angleFromLeastSq()
{
  calcPointParameters();
  angle_ += angleIncrement();
}

double Line::angleIncrement()
{
  const std::vector<double> &a = p_params_.a;
  const std::vector<double> &ap = p_params_.ap;
  const std::vector<double> &app = p_params_.app;
  const std::vector<double> &b = p_params_.b;
  const std::vector<double> &bp = p_params_.bp;
  const std::vector<double> &bpp = p_params_.bpp;
  const std::vector<double> &c = p_params_.c;
  const std::vector<double> &s = p_params_.s;

  double numerator = 0; 
  double denominator = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    numerator += (b[i] * ap[i] - a[i] * bp[i]) / pow(b[i], 2);
    denominator += ((app[i] * b[i] - a[i] * bpp[i]) * b[i] - 
                    2 * (ap[i] * b[i] - a[i] * bp[i]) * bp[i]) / pow(b[i], 3);
  }
  return -(numerator/denominator);
}

void Line::calcCovariance()
{
  covariance_[0] = p_rr_;

  const std::vector<double> &a = p_params_.a;
  const std::vector<double> &ap = p_params_.ap;
  const std::vector<double> &app = p_params_.app;
  const std::vector<double> &b = p_params_.b;
  const std::vector<double> &bp = p_params_.bp;
  const std::vector<double> &bpp = p_params_.bpp;
  const std::vector<double> &c = p_params_.c;
  const std::vector<double> &s = p_params_.s;

  double G = 0;
  double A = 0;
  double B = 0;
  double r, phi;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    r = r_data_.ranges[indices_[i]]; // range
    phi = c_data_.bearings[indices_[i]]; // bearing
    G += ((app[i] * b[i] - a[i] * bpp[i]) * b[i] - 2 * (ap[i] * b[i] - a[i] * bp[i]) * bp[i]) / pow(b[i], 3);
    A += 2 * r * sin(angle_ - phi) / b[i];
    B += 4 * pow(r, 2) * pow(sin(angle_ - phi), 2) / b[i];
  }
  covariance_[1] = p_rr_ * A / G;
  covariance_[2] = covariance_[1];
  covariance_[3] = pow(1.0 / G, 2) * B;
}

void Line::calcPointCovariances()
{
  point_covs_.clear();
  double r, phi, var_r, var_phi;
  for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
  {
    r = r_data_.ranges[*cit]; // range
    phi = c_data_.bearings[*cit]; // bearing
    var_r = params_.range_var; // range variance
    var_phi = params_.bearing_var; // bearing variance
    boost::array<double, 4> Q; 
    Q[0] = pow(r, 2) * var_phi * pow(sin(phi), 2) + var_r * pow(cos(phi), 2);
    Q[1] = -pow(r, 2) * var_phi * sin(2 * phi) / 2.0 + var_r * sin(2 * phi) / 2.0;
    Q[2] = Q[1]; 
    Q[3] = pow(r, 2) * var_phi * pow(cos(phi), 2) + var_r * pow(sin(phi), 2);
    point_covs_.push_back(Q);
  }
}

void Line::calcPointParameters()
{
  p_params_.a.clear();
  p_params_.ap.clear();
  p_params_.app.clear();
  p_params_.b.clear();
  p_params_.bp.clear();
  p_params_.bpp.clear();
  p_params_.c.clear();
  p_params_.s.clear();

  double r, phi, var_r, var_phi;
  double a, ap, app, b, bp, bpp, c, s;
  for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
  {
    r = r_data_.ranges[*cit]; // range
    phi = c_data_.bearings[*cit]; // bearing
    var_r = params_.range_var; // range variance
    var_phi = params_.bearing_var; // bearing variance
    c = cos(angle_ - phi);
    s = sin(angle_ - phi);
    a = pow(r * c - radius_, 2);
    ap = -2 * r * s * (r * c - radius_);
    app = 2 * pow(r, 2) * pow(s, 2) - 2 * r * c * (r * c - radius_);
    b = var_r * pow(c, 2) + var_phi * pow(r, 2) * pow(s, 2);
    bp = 2 * (pow(r, 2) * var_phi - var_r) * c * s;
    bpp = 2 * (pow(r, 2) * var_phi - var_r) * (pow(c, 2) - pow(s, 2));
    p_params_.a.push_back(a);
    p_params_.ap.push_back(ap);
    p_params_.app.push_back(app);
    p_params_.b.push_back(b);
    p_params_.bp.push_back(bp);
    p_params_.bpp.push_back(bpp);
    p_params_.c.push_back(c);
    p_params_.s.push_back(s);
  }
}

void Line::calcPointScalarCovariances()
{
  point_scalar_vars_.clear();
  double P;
  double inverse_P_sum = 0;
  for (std::vector<boost::array<double, 4> >::const_iterator cit = point_covs_.begin();
       cit != point_covs_.end(); ++cit)
  {
    P = (*cit)[0] * pow(cos(angle_), 2) + 2 * (*cit)[1] * sin(angle_) * cos(angle_) +
        (*cit)[3] * pow(sin(angle_), 2);
    inverse_P_sum += 1.0 / P;
    point_scalar_vars_.push_back(P);
  }
  p_rr_ = 1.0 / inverse_P_sum;
}

void Line::radiusFromLeastSq()
{
  radius_ = 0;
  double r, phi;
  for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
  {
    r = r_data_.ranges[*cit]; // range
    phi = c_data_.bearings[*cit]; // bearing
    radius_ += r * cos(angle_ - phi) / point_scalar_vars_[cit - indices_.begin()]; // cit to index
  }
  
  radius_ *= p_rr_;
}

} // namespace line_extraction
