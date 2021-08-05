/**
 *  MIT License
 *
 *  Copyright (c) 2019 Yuya Kudo
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a copy
 *  of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 *  furnished to do so, subject to the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

#ifndef LIB_INCLUDE_NODE_SIMPLENODELIST_H_
#define LIB_INCLUDE_NODE_SIMPLENODELIST_H_

#include <Node/NodeListBase.h>

#include <limits>

namespace planner {
/**
 *  using std::vector and the NN and NBHD are solved by linear search.
 */
class SimpleNodeList : public base::NodeListBase {
  using NodePtr = std::shared_ptr<Node>;

 public:
  explicit SimpleNodeList(const uint32_t &dim);
  ~SimpleNodeList();
  void add(const NodePtr &node);
  void init();
  int getSize();
  NodePtr searchNN(const NodePtr &node);
  std::vector<NodePtr> searchNBHD(const NodePtr &node, const double &radius);
  std::vector<NodePtr> searchLeafs();

 private:
  std::vector<NodePtr> list_;
};
}  // namespace planner

#endif /* LIB_INCLUDE_NODE_SIMPLENODELIST_H_ */
