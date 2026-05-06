# V2AIX replay demo

Minimal compose setup to replay [V2AIX](https://v2aix.ika.rwth-aachen.de/) ROS 2 bags from stationary recordings, including the ETSI ITS conversion node, to visualize messages in RViz.

> [!IMPORTANT]  
> Mobile recordings are not supported yet, even though it would be conceivable to visualize these. To support this properly, it would require an additional TF broadcaster to indicate the ego-vehicle position within the UTM Frame (based on the `NavSatFix` messages in the bag) and move the map display accordingly.

## Configuration

The `.env` file controls:
- `BAG_DIR` (host path containing the bag-files)
- `BAG_PATH` (actual bagfile relative to `BAG_DIR`)
- `RVIZ_LAT` and `RVIZ_LON` (AerialMap-Display position)

## Quick start

Run the composition (assuming you are in the `utils/v2aix` directory of this repository):
```bash
docker compose up
```

If RViz does not show up, ensure X11 permissions are set (e.g. `xhost +local:`) and `DISPLAY` is available.