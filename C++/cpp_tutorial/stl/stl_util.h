#pragma once

namespace stl_util {
/**
 * @brief This print function is valid for std::set, std::vector, std::list
 *                it not worked for std::map
 * @ref http://www.java2s.com/Tutorial/Cpp/0260__template/templatefunctiontoprintelementsofanSTLcontainer.htm
 **/
template <typename T>
void PrintSTLContainer(T const &container)
{
  typename T::const_iterator pos;                  // iterator to iterate over coll
  typename T::const_iterator end(container.end()); // end position

  for (pos = container.begin(); pos != end; ++pos) {
    std::cout << *pos << " ";
  }
  std::cout << std::endl;
}

} // namespace stl_util