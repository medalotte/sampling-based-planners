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

#ifndef LIB_INCLUDE_NODE_NODELISTBASE_H_
#define LIB_INCLUDE_NODE_NODELISTBASE_H_

#include <Node/Node.h>

namespace planner {
namespace base {
/**
 *  Base class of node list for sampling-based planners
 */
class NodeListBase {
  using NodePtr = std::shared_ptr<Node>;

 public:
  const uint32_t DIM;
  explicit NodeListBase(const uint32_t &_dim);
  virtual ~NodeListBase();
  virtual void add(const NodePtr &node) = 0;
  virtual void init() = 0;
  virtual int getSize() = 0;
  virtual NodePtr searchNN(const NodePtr &node) = 0;
  virtual std::vector<NodePtr> searchNBHD(const NodePtr &node, const double &radius) = 0;
  virtual std::vector<NodePtr> searchLeafs() = 0;
};
}  // namespace base
}  // namespace planner

#endif /* LIB_INCLUDE_NODE_NODELISTBASE_H_ */
