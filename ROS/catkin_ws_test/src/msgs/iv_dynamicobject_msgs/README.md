## 动态障碍物消息定义
`moving_target.msg`
**所有坐标都为全局坐标系下，即在地图上的坐标**
int32 ID_number 目标ID
Points center_point 目标中心点坐标
int32 line_num   构成目标形状多边形边数（目前只是用矩形，为4条边）
Points[] line_point    构成目标形状多边形顶点坐标
float32 object_high   目标高度
int32 object_type      目标类型（小车、大车等）
bool is_updated   目标是否更新
int32  tracked_times  目标跟踪次数
int32  dangerous_level 目标危险程度
int32  history_num 目标历史数（暂时没用）
History_traj[] history_traj  目标历史轨迹
int32  predict_num   目标运动预测数
Predict_traj[] predict_traj 目标运动预测轨迹
