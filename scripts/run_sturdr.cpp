#include <string>

#include "sturdr/sturdr.hpp"

int main(int argc, char *argv[]) {
  std::string yaml_filename;
  if (argc > 1) {
    yaml_filename = argv[1];
  } else {
    yaml_filename = "config/gps_l1ca_rcvr.yaml";
  }

  sturdr::SturDR<int8_t> rcvr(yaml_filename);
  rcvr.Run();

  return 0;
}