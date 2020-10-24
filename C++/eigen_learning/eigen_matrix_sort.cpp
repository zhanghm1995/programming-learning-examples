/*
 * @Author: Haiming Zhang
 * @Email: zhanghm_1995@qq.com
 * @Date: 2020-10-24
 * @References: Sequence operation in Eigen
 * @Description: Learn how to sort matrix in Eigen
 *                               对eigen矩阵进行排序，得到排序后的索引；
 */

#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>

#include <Eigen/Dense>
using namespace std;

bool compare_head(const Eigen::VectorXd& lhs, const Eigen::VectorXd& rhs)
{
    return lhs(0) < rhs(0);
}

Eigen::MatrixXd sorted_rows_by_head(Eigen::MatrixXd A)
{
    std::vector<Eigen::VectorXd> vec;
    for (int64_t i = 0; i < A.rows(); ++i)
        vec.push_back(A.row(i));

    std::sort(vec.begin(), vec.end(), &compare_head);

    for (int64_t i = 0; i < A.rows(); ++i)
        A.row(i) = vec[i];

    return A;
}

template <typename T>
vector<size_t> sort_indexes(const vector<T> &v) {

  // initialize original index locations
  vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values 
  stable_sort(idx.begin(), idx.end(),
       [&v](size_t i1, size_t i2) {return v[i1] < v[i2];});

  return idx;
}


int main()
{
  Eigen::MatrixXd distance_matrix = Eigen::MatrixXd::Zero(5, 4);
  distance_matrix<<61.5649, 83.9368 ,90.2591, 82.0116,
                   29.0051, 7.89539, 0.309848, 9.7817,
                   0.245993, 22.4503, 28.4495, 20.6276,
                   22.4596, 0.233815, 7.83312, 1.75816,
                   22.4596, 22.4503 , 7.83312, 0.233815;

  Eigen::MatrixXd distance_matrix1 = distance_matrix.transpose();
  cout<<distance_matrix1<<endl;

  std::vector<double> distance_matrix_content(distance_matrix1.data(), 
                                              distance_matrix1.data() + distance_matrix1.size());

  // std::sort(distance_matrix_content.begin(), distance_matrix_content.end());
  for (const auto& v : distance_matrix_content) {
    cout<<v<<endl;
  }

  auto index_1d = sort_indexes(distance_matrix_content);
  for (const auto& i : index_1d) {
    cout<<i<<endl;
  }

  std::vector<int> detection_id_matches_to_tracking_id(distance_matrix.rows(), -1);
  std::vector<int> tracking_id_matches_to_detection_id(distance_matrix.cols(), -1);

  int num_detections(distance_matrix.rows()), num_tracks(distance_matrix.cols());

  std::vector<std::pair<int, int> > matched_indices;
  for (const auto& o : index_1d) {
    int detection_id = (int)(std::floor(o / num_tracks));
    int track_id = (int)(o % num_tracks);
    cout<<detection_id<<" "<<track_id<<endl;
    if (tracking_id_matches_to_detection_id[track_id] == -1 &&
        detection_id_matches_to_tracking_id[detection_id] == -1) {
          tracking_id_matches_to_detection_id[track_id] = detection_id;
          detection_id_matches_to_tracking_id[detection_id] = track_id;
          matched_indices.push_back(std::make_pair(detection_id, track_id));
        }
  }

  cout<<"==================="<<endl;
  for (const auto& d : matched_indices) {
    cout<<d.first<<" "<<d.second<<endl;
  }

  cout<<"==================="<<endl;
  std::vector<int> matched_indices2 = std::vector<int>(distance_matrix.rows(), -1);
  for (const auto& d : matched_indices) {
    matched_indices2[d.first] = d.second;
  }
  for (const auto& d : matched_indices2) {
    cout<<d<<endl;
  }
}
