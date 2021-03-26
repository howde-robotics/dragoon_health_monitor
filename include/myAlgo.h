#pragma once

#include <vector>

namespace howde {

/**
 * @brief this class does what
 *  
 */
class MyAlgo
{
public:
  /**
   * @brief Construct a new My Algo object
   * 
   * @param param this will be stored inside someVec 
   */
	MyAlgo(int param);

  /**
   * @brief explain this code
   * 
   * @param x it is this thing with this unit
   * @param y that other stuff with another unit
   * @return int that calculates this stuff
   */
  int run(int x, int y);

  /**
   * @brief helper function
   * 
   * @param z of this param
   * @return int bla bla
   */
  int someFunction(int z);
private:
  std::vector<int> someVec; ///< bla bla
};

} // namespace howde