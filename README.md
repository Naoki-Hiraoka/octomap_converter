# octomap_converter

## octomap_limit_filter
octomapのうち`~center_frame`を中心とする半径`~radius`のBoxの内部にある部分のみを抽出する

### Subscribing Topics
* `~input` (`octomap_msgs/Octomap`)

#### Publishing Topics
* `~output` (`octomap_msgs/Octomap`)

#### Parameters
* `~center_frame` (String, default: `""`)
* `~radius` (Double, default: `1.0`[m])

#### Required TF
* frame_id of `~input` -> `~center_frame`

## octomap_frame_converter
octomapのframe_idを`~target_frame`に変える

### Subscribing Topics
* `~input` (`octomap_msgs/Octomap`)

#### Publishing Topics
* `~output` (`octomap_msgs/OctomapWithPose`)

#### Parameters
* `~target_frame` (String, default: `"odom"`)

#### Required TF
* frame_id of `~input` -> `~target_frame`

## octomap_to_occupancygrid
octomapからocuppancy gridに射影する. 天井や床を削除するため，octomapの座標系で，`~limit_frames`の中で最も低いZ座標を基準として，Z座標が`~limit_max_z`と`~limit_min_z`の間にあるboxelのみを射影する.

### Subscribing Topics
* `~input` (`octomap_msgs/Octomap`)

#### Publishing Topics
* `~output` (`nav_msgs::OccupancyGrid`)

#### Parameters
* `~limit_frames` (Sequence of String, default: `[]`)
* `~limit_min_z` (Double, default: `0.15`[m])
* `~limit_max_z` (Double, default: `2.0`[m])

## octomap_speckle_remover
octomapの中でsize以下の大きさのoccupied cellの塊を削除する

### Subscribing Topics
* `~input` (`octomap_msgs/Octomap`)

#### Publishing Topics
* `~output` (`octomap_msgs/Octomap`)

#### Parameters
* `~size` (Int, default: `1`)
