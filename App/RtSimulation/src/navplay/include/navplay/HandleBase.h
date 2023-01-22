/**
* @file HandleBase.h in InsCubeDecoder
* @author rebeater
* @comment
* Create on 3/14/22 9:20 PM
* @version 1.0
**/

#ifndef INSCUBEDECODER__HANDLEBASE_H_
#define INSCUBEDECODER__HANDLEBASE_H_
#include <string>
namespace InsCube{
class HandleBase {
 private:
  std::string DeviceName;

 public:
  const std::string &GetDeviceName() const;
  HandleBase() = default;
  explicit HandleBase(std::string device) : DeviceName(std::move(device)) {};//=default;
};
class CptHandle : public HandleBase {
 public:
  CptHandle() : HandleBase("SPAN CPT") {}
};
}
#endif //INSCUBEDECODER__HANDLEBASE_H_
