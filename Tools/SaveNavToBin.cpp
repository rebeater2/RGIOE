/**
* @file SaveNavToBin.cpp in LooselyCouple2020_cpp
* @author rebeater
* @comment
* Create on 3/31/22 2:58 PM
* @version 1.0
**/

#include <string>
#include "FileIO.h"
#include "Earth.h"
struct NavBin {
  double gpst;
  double pos[3];
  double horiz[2];
  double vn[3];
  double atti[3];
  double pos_std[3];
  double vn_std[3];
  double atti_std[3];
};
void ConvertNavToBin(const NavOutput &nav, NavBin &bin);
/**
 * Usage: SaveNavToBin filename.nav
 */
int main(int argc, char **argv) {
  if (argc < 0) return 1;
  std::string nav_path = argv[1];
  NavReader nav_reader{nav_path,NavFileFormat::NavBinary};
  NavOutput nav;
  std::ofstream ofs(nav_path+".for_ima.bin");
  NavBin bin;
  while (nav_reader.IsOk()) {
	nav_reader.ReadNext(nav);
	std::cout << nav<<'\n';
	ConvertNavToBin(nav,bin);
	ofs.write((char *)&bin,sizeof(bin));
  }
}

void ConvertNavToBin(const NavOutput &nav, NavBin &bin) {
  static double base_position[3] = {0, 0, 0};
  bin.gpst = nav.gpst;
  bin.pos[0] = nav.lat;
  bin.pos[1] = nav.lon;
  bin.pos[2] = nav.height;
  auto delta_d = Earth::Instance().distance(
	  nav.lat * _deg, nav.lon * _deg,
	  base_position[0] * _deg, base_position[1] * _deg);
  bin.horiz[0] = delta_d[1];
  bin.horiz[1] = delta_d[0];
  for (int i = 0; i < 3; i++) {
	bin.vn[i] = nav.vn[i];
	bin.atti[i] = nav.atti[i];
	bin.pos_std[i] = nav.pos_std[i];
	bin.atti_std[i] = nav.atti_std[i];
	bin.vn_std[i] = nav.vn_std[i];
  }
};
