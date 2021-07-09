//
// Created by rebeater on 6/18/21.
//

#include <pangolin/pangolin.h>
#include <Eigen/Core>
#include <unistd.h>
#include <fstream>
#include "Convert.h"
#include "NavLog.h"

using namespace std;
using namespace Eigen;

// path to trajectory file
//string trajectory_file = "/media/rebeater/hd_data2/workspace/学习资料/slam/slambook2/ch3/examples/trajectory.txt";

void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>>);
Eigen::Quaterniond Euler2Quaternion(const Vec3d &euler)
{
  //DONE check by little fang in 20190712
  return (Eigen::AngleAxisd(euler[2], Eigen::Vector3d::UnitZ()) *
	  Eigen::AngleAxisd(euler[1], Eigen::Vector3d::UnitY()) *
	  Eigen::AngleAxisd(euler[0], Eigen::Vector3d::UnitX()));
}
int main(int argc, char **argv) {
  if (argc < 2) {
	cout << "usage showTraj **.nav" << endl;
  }

  vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses;
  string trajectory_file = argv[1];
  ifstream fin(trajectory_file);
  if (!fin) {
	cout << "cannot find trajectory file at " << trajectory_file << endl;
	return 1;
  }
  string buffer;

  getline(fin, buffer);
  stringstream ss{buffer};
  double week, time, tx, ty, tz, vx, vy, vz, ax, ay, az, gb[3], ab[3], gs[3], as[3], qx, qy, qz, qw;
  ss >> week >> time >> tx >> ty >> tz >> vx >> vy >> vz >> ax >> ay >> az;
  Vec3d _pos{tx * _deg, ty * _deg, tz};
  Vec3d pos0 = Convert::lla_to_xyz(_pos);
  logi << tx << " " << ty << " " << tz << "\t" << pos0.transpose() << endl;
  Vec3d pos;
  while (!fin.eof()) {  //fin.eof()判断文件是否为空
	getline(fin, buffer);
	stringstream ss1{buffer};
	ss1 >> week >> time >> tx >> ty >> tz >> vx >> vy >> vz >> ax >> ay >> az;
	_pos = {tx * _deg, ty * _deg, tz};
	pos = Convert::lla_to_xyz(_pos);
	pos -= pos0;
	Vec3d atti{az * _deg, ay * _deg, ax * _deg};
	Eigen::Quaternion<double> q=Euler2Quaternion(atti);
	Isometry3d Twr(q);  //变换矩阵的旋转部分
//	logi << tx << " " << ty << " " << tz << "\t" << pos0.transpose() << endl;
	Twr.pretranslate(Vector3d(pos[0], pos[1], pos[2]));//变换矩阵的平移部分
	poses.push_back(Twr);
  }
  cout << "read total " << poses.size() << " pose entries" << endl;

  // draw trajectory in pangolin
  DrawTrajectory(poses);
  return 0;
}

/*******************************************************************************************/
void DrawTrajectory(vector<Isometry3d, Eigen::aligned_allocator<Isometry3d>> poses) {
  // create pangolin window and plot the trajectory
  pangolin::CreateWindowAndBind("Trajectory Viewer", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState s_cam(
	  pangolin::ProjectionMatrix(1024, 768, 500, 500, 512, 389, 0.1, 1000),
	  pangolin::ModelViewLookAt(0, -0.1, -1.8, 0, 0, 0, 0.0, -1.0, 0.0)
  );

  pangolin::View &d_cam = pangolin::CreateDisplay()
	  .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
	  .SetHandler(new pangolin::Handler3D(s_cam));

  while (pangolin::ShouldQuit() == false) {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	d_cam.Activate(s_cam);
	glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
	glLineWidth(2);
	for (size_t i = 0; i < poses.size(); i++) {
	  // 画每个位姿的三个坐标轴
	  Vector3d Ow = poses[i].translation();
	  Vector3d Xw = poses[i] * (0.1 * Vector3d(1, 0, 0));
	  Vector3d Yw = poses[i] * (0.1 * Vector3d(0, 1, 0));
	  Vector3d Zw = poses[i] * (0.1 * Vector3d(0, 0, 1));
	  glBegin(GL_LINES);
	  glColor3f(1.0, 0.0, 0.0);
	  glVertex3d(Ow[0], Ow[1], Ow[2]);
	  glVertex3d(Xw[0], Xw[1], Xw[2]);
	  glColor3f(0.0, 1.0, 0.0);
	  glVertex3d(Ow[0], Ow[1], Ow[2]);
	  glVertex3d(Yw[0], Yw[1], Yw[2]);
	  glColor3f(0.0, 0.0, 1.0);
	  glVertex3d(Ow[0], Ow[1], Ow[2]);
	  glVertex3d(Zw[0], Zw[1], Zw[2]);
	  glEnd();
	}
	// 画出连线
	for (size_t i = 0; i < poses.size(); i++) {
	  glColor3f(0.0, 0.0, 0.0);
	  glBegin(GL_LINES);
	  auto p1 = poses[i], p2 = poses[i + 1];
	  glVertex3d(p1.translation()[0], p1.translation()[1], p1.translation()[2]);
	  glVertex3d(p2.translation()[0], p2.translation()[1], p2.translation()[2]);
	  glEnd();
	}
	pangolin::FinishFrame();
//	usleep(5000);   // sleep 5 ms
  }
}