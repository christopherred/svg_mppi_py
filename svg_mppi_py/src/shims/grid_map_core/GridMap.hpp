#pragma once
#include <Eigen/Dense>
#include <string>
#include <map>
#include <cmath>
#include <vector>

// 模拟 grid_map 命名空间，欺骗原有的 C++ 算法代码
namespace grid_map {

// 基础数据结构模拟
struct Position {
    double x, y;
    Position(double _x = 0, double _y = 0) : x(_x), y(_y) {}
    double operator[](int i) const { return (i==0) ? x : y; }
    double& operator[](int i) { return (i==0) ? x : y; }
};

struct Length {
    double x, y;
    Length(double _x = 0, double _y = 0) : x(_x), y(_y) {}
    double x_len() const { return x; }
    double y_len() const { return y; }
    double operator()(int i) const { return (i==0) ? x : y; }
};

struct Index {
    int x, y;
    Index(int _x = 0, int _y = 0) : x(_x), y(_y) {}
    int x_idx() const { return x; }
    int y_idx() const { return y; }
    int operator[](int i) const { return (i==0) ? x : y; }
};

struct Size {
    int x, y;
    Size(int _x = 0, int _y = 0) : x(_x), y(_y) {}
    int prod() const { return x * y; }
    int operator()(int i) const { return (i==0) ? x : y; }
};

// 核心 GridMap 类模拟
class GridMap {
public:
    using Matrix = Eigen::MatrixXf;

    GridMap() {}
    // 原代码用 vector<string> 初始化，兼容一下
    GridMap(const std::vector<std::string>& layers) {}

    // 设置几何参数
    void setGeometry(const Length& length, double resolution, const Position& position) {
        length_ = length;
        resolution_ = resolution;
        position_ = position;
        size_ = Size(
            static_cast<int>(std::round(length.x / resolution)),
            static_cast<int>(std::round(length.y / resolution))
        );
    }
    
    // 设置 FrameId (原代码有调用，虽然这里没用)
    void setFrameId(const std::string& id) { frame_id_ = id; }

    // 核心 API: 添加数据 (供 Python 调用)
    void add(const std::string& layer, const Matrix& data) {
        layers_[layer] = data;
        if (size_.x == 0) {
            size_ = Size(data.rows(), data.cols());
        }
    }

    // 核心 API: 获取数据引用 (MPC Base 会用到)
    Matrix& get(const std::string& layer) {
        if (layers_.find(layer) == layers_.end()) {
            layers_[layer] = Matrix::Zero(size_.x, size_.y);
        }
        return layers_[layer];
    }
    
    const Matrix& get(const std::string& layer) const {
        return layers_.at(layer);
    }

    // 核心 API: 检查点是否在地图内
    bool isInside(const Position& pos) const {
        double half_x = length_.x / 2.0;
        double half_y = length_.y / 2.0;
        // 假设无旋转，简单 Box check
        return (pos.x >= position_.x - half_x && pos.x <= position_.x + half_x &&
                pos.y >= position_.y - half_y && pos.y <= position_.y + half_y);
    }

    // 核心 API: 获取某点的值
    float atPosition(const std::string& layer, const Position& pos) const {
        Index idx;
        if (!getIndex(pos, idx)) return 0.0f;
        if (layers_.find(layer) == layers_.end()) return 0.0f;
        // 注意 Eigen 矩阵索引 (row, col)
        return layers_.at(layer)(idx.x, idx.y);
    }

    // 坐标转索引 (根据 grid_map_core 逻辑，通常中心对齐)
    bool getIndex(const Position& pos, Index& index) const {
        if (!isInside(pos)) return false;
        // 简化计算：原点在中心
        // GridMap 坐标系：通常 x 向前，y 向左。
        // Index (0,0) 通常对应左上角或根据迭代器定义。
        // 这里为了简单，假设 (0,0) 是 (centerX - lenX/2, centerY - lenY/2)
        double dx = pos.x - (position_.x - length_.x / 2.0);
        double dy = pos.y - (position_.y - length_.y / 2.0);
        
        index.x = static_cast<int>(dx / resolution_);
        index.y = static_cast<int>(dy / resolution_);

        // 边界保护
        if (index.x < 0) index.x = 0; if (index.x >= size_.x) index.x = size_.x - 1;
        if (index.y < 0) index.y = 0; if (index.y >= size_.y) index.y = size_.y - 1;
        
        return true;
    }

    double getResolution() const { return resolution_; }
    Length getLength() const { return length_; }
    Size getSize() const { return size_; }

    // 线性索引转二维索引 (用于并行计算循环)
    static Index getIndexFromLinearIndex(int i, const Size& size) {
        // GridMap 是列优先还是行优先取决于实现，这里假设行优先
        return {i / size.y, i % size.y}; 
    }

    // 索引转坐标
    void getPosition(const Index& idx, Position& pos) const {
        pos.x = (position_.x - length_.x / 2.0) + (idx.x + 0.5) * resolution_;
        pos.y = (position_.y - length_.y / 2.0) + (idx.y + 0.5) * resolution_;
    }

private:
    std::map<std::string, Matrix> layers_;
    Length length_;
    double resolution_ = 0.1;
    Position position_;
    Size size_;
    std::string frame_id_;
};

} // namespace grid_map