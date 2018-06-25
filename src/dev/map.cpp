#ifndef SFO_MAP_H
#define SFO_MAP_H

namespace SFO {
	class Map {
	public:
		Map();

		void addKeyFrame(KeyFrame *pKF);
		void addMapPoint(MapPoint *pMP);
		void eraseKeyFrame(KeyFrame *pKF);
		void eraseMapPoint(MapPoint *pMP);
		void setReferenceMapPoints(const std::std::vector<MapPoint *> &vpMPs);
		void informNewBigChange();
		void getLastBigChangeIdx();

		std::vector<KeyFrame *> getAllKeyFrames();
		std::vector<MapPoint *> getAllMapPoints();
		std::vector<MapPoint *> getReferenceMapPoints();

		long unsigned int MapPointsInMap();
		long unsigned KeyFramesInMap();

		long unsigned int getMaxKFid();

		void clear();

		std::vector<KeyFrame *> mvpKeyFrameOrigins;

		std::mutex mMutexMapUpdate
	};

} // namespace SFO

#endif 