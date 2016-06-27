#include "ofApp.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <common/common.h>
#include <pcl/common/time.h>

//--------------------------------------------------------------
void ofApp::setup()
{
	std::chrono::seconds five_sec(3);
	static_time_to_snapshot_ = five_sec;
	cloudColors[0].set(255, 0, 0);
	cloudColors[1].set(0, 255, 0);
	cloudColors[2].set(0, 0, 255);
	cloudColors[3].set(255, 255, 0);

	// SENSORS ##############
	recon::SensorFactory sensorFac;

	auto nSensors = sensorFac.checkConnectedDevices(true);
	for (int i = 0; i < nSensors; i++)
	{
		sensor_list_.push_back(sensorFac.createPclOpenNI2Grabber());
		detected_sphere_[sensor_list_.back()->getId()].setResolution(6);
		calib_positions_[sensor_list_.back()->getId()] = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>(pcl::PointCloud<pcl::PointXYZ>());
	}

	// UI ############
	ui_.setup();
	ui_.add(bgSl_.setup(background_));

	calibResetBtn_.addListener(this, &ofApp::reset_calibration);
	performEstimBtn_.addListener(this, &ofApp::performICPTransformationEstimation);
	ui_.add(calibResetBtn_.setup("Reset Calibration"));
	ui_.add(performEstimBtn_.setup("Perform Calibration"));

	saveCalibrationBtn_.addListener(this, &ofApp::saveCalibrationToFile);
	loadCalibrationBtn_.addListener(this, &ofApp::loadCalibrationFromFile);
	ui_.add(saveCalibrationBtn_.setup("Save calibration"));
	ui_.add(loadCalibrationBtn_.setup("Load calibration"));

	filteringParams_.setName("Filtering Parameters");
	filteringParams_.add(resolution_);
	filteringParams_.add(passMin_);
	filteringParams_.add(passMax_);
	ui_.add(filteringParams_);

	trackingParams_.setName("Tracking Parameters");
	trackingParams_.add(trackingEnabled_);
	trackingParams_.add(minR_);
	trackingParams_.add(maxR_);
	trackingParams_.add(samples_);
	trackingParams_.add(error_);
	trackingParams_.add(percent_);
	trackingParams_.add(meanSamples_);
	trackingParams_.add(movementThreshold_);
	ui_.add(trackingParams_);

	icpParams_.setName("ICP Parameters");
	icpParams_.add(icpDistanceThreshold_);
	icpParams_.add(icpIterations_);
	ui_.add(icpParams_);



	// CAMERA ##############

	cam_.rotate(180, 0, 1, 0);
	cam_.setFarClip(100000);

	player_.load("click.mp3");
}

//--------------------------------------------------------------
void ofApp::update()
{
	bool take_snapshot = true;
	// for each sensor
	for (auto& sensor : sensor_list_)
	{
		// get current point cloud
		auto cloud = sensor->getCloudSource()->getOutputCloud();
		if (cloud != nullptr)
		{
			// downsample cloud for searching sphere
			recon::CloudPtr cloud_downsampled(new recon::Cloud());
			downsample(cloud, cloud_downsampled, resolution_);

			// remove background
			recon::CloudPtr cloud_wo_back(new recon::Cloud());
			removeBackground(cloud_downsampled, cloud_wo_back, passMin_, passMax_, false);

			// find sphere
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			if (trackingEnabled_)
			{
				Eigen::VectorXf s_param;

				findSphere(cloud_wo_back, inliers, s_param);

				if (inliers->indices.size() > 0 && s_param.size() == 4)
				{
					auto sphere_r = s_param.w();
					auto sphere_x = s_param.x();
					auto sphere_y = s_param.y();
					auto sphere_z = s_param.z();

					sphere_detected_[sensor->getId()] = true;

					// get in- and outlier cloud
					recon::CloudPtr in_cloud(new recon::Cloud());
					recon::CloudPtr out_cloud(new recon::Cloud());
					extractInOutliers(cloud_wo_back, inliers, in_cloud, out_cloud);
					// inliers mesh to draw
					createOfMeshFromPoints(in_cloud, ofColor(0, 255, 0, 255), inliers_mesh_[sensor->getId()]);
					auto c = cloudColors[sensor->getId()];
					c.a = 64;
					// outliers mesh to draw
					createOfMeshFromPoints(out_cloud, c, mesh_map_[sensor->getId()]);

					// calculate mean radius of calib target
					meanR_[sensor->getId()] = approxRollingAverage(meanR_[sensor->getId()], sphere_r * 1000, meanSamples_);
					detected_sphere_[sensor->getId()].setRadius(meanR_[sensor->getId()]);

					// calculate new mean position
					last_mean_pos_[sensor->getId()] = ofVec3f(meanX_[sensor->getId()], meanY_[sensor->getId()], meanZ_[sensor->getId()]);
					meanX_[sensor->getId()] = approxRollingAverage(meanX_[sensor->getId()], sphere_x * 1000, meanSamples_);
					meanY_[sensor->getId()] = approxRollingAverage(meanY_[sensor->getId()], sphere_y * 1000, meanSamples_);
					meanZ_[sensor->getId()] = approxRollingAverage(meanZ_[sensor->getId()], sphere_z * 1000, meanSamples_);
					detected_sphere_location_[sensor->getId()].set(meanX_[sensor->getId()], meanY_[sensor->getId()], meanZ_[sensor->getId()]);
				}
				// if no sphere was found
				else
				{
					sphere_detected_[sensor->getId()] = false;
					// make ofMesh for displaying
					ofMesh mesh;
					auto c = cloudColors[sensor->getId()];
					c.a = 64;
					createOfMeshFromPoints(cloud_wo_back, c, mesh);
					mesh_map_.erase(sensor->getId());
					mesh_map_.insert(std::pair<int, ofMesh>(sensor->getId(), mesh));
				}


				// when tracking target has moved more than a cm in one of the directions
				if (std::fabs(last_mean_pos_[sensor->getId()].x - meanX_[sensor->getId()]) >= movementThreshold_
					|| std::fabs(last_mean_pos_[sensor->getId()].y - meanY_[sensor->getId()]) >= movementThreshold_
					|| std::fabs(last_mean_pos_[sensor->getId()].z - meanZ_[sensor->getId()]) >= movementThreshold_)
				{
					take_snapshot = false;
				}
			}
			// if tracking is disabled
			else
			{
				sphere_detected_[sensor->getId()] = false;
				// make ofMesh for displaying
				ofMesh mesh;
				createOfMeshFromPoints(cloud_wo_back, mesh);
				mesh_map_.erase(sensor->getId());
				mesh_map_.insert(std::pair<int, ofMesh>(sensor->getId(), mesh));
			}
		}
	}
	// take snapshot if conditions where met
	if (take_snapshot && trackingEnabled_)
	{
		// check elapsed time in static pose
		auto now = std::chrono::steady_clock::now();
		auto time_static = now - static_since_;
		auto time_since_last_snap = now - last_snap_;

		// if bigger than time to snapshot, do snapshot
		if (time_static >= static_time_to_snapshot_ && time_since_last_snap >= static_time_to_snapshot_)
		{
			for (auto& sensor : sensor_list_)
			{
				pcl::PointXYZ calib_point(detected_sphere_location_[sensor->getId()].x,
				                          detected_sphere_location_[sensor->getId()].y,
				                          detected_sphere_location_[sensor->getId()].z);
				calib_positions_[sensor->getId()]->push_back(calib_point);
				last_snap_ = std::chrono::steady_clock::now();
				performICPTransformationEstimation();
			}

			player_.play();
		}
	}
	else
	{
		// reset timer
		static_since_ = std::chrono::steady_clock::now();
	}
}

//--------------------------------------------------------------
void ofApp::draw()
{
	ofBackground(background_);

	ofDrawBitmapString("fps: " + std::to_string(ofGetFrameRate()) + " calpos: " + std::to_string(calib_positions_[0]->size()), 10, 10);

	cam_.begin();
	ofEnableDepthTest();
	for (auto& sensor : sensor_list_)
	{
		ofPushMatrix();
		auto ext = sensor->getDepthExtrinsics();
		auto translation = toOfVector3(*ext->getTranslation());
		auto rotation = toOfQuaternion(*ext->getRotation());
		ofVec3f qaxis;
		float qangle;
		rotation.getRotate(qangle, qaxis);
		ofTranslate(translation);
		ofRotate(qangle, qaxis.x, qaxis.y, qaxis.z);

		mesh_map_[sensor->getId()].drawVertices();

		if (trackingEnabled_)
		{
			if (sphere_detected_[sensor->getId()])
			{
				inliers_mesh_[sensor->getId()].draw();

				ofPushMatrix();
				ofPushStyle();
				ofSetColor(cloudColors[sensor->getId()]);
				ofTranslate(detected_sphere_location_[sensor->getId()]);
				detected_sphere_[sensor->getId()].drawWireframe();
				ofPopMatrix();
				ofPopStyle();
			}

			for (auto& p : *calib_positions_[sensor->getId()])
			{
				ofPushStyle();
				ofSetColor(cloudColors[sensor->getId()]);
				ofDrawBox(p.x, p.y, p.z, 30);
				ofPopStyle();
			}
		}
		ofPopMatrix();
	}


	cam_.end();

	ofDisableDepthTest();
	ui_.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key)
{
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key)
{
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y)
{
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg)
{
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo)
{
}

void ofApp::reset_calibration()
{
	for (auto& sensor : sensor_list_)
	{
		calib_positions_[sensor->getId()]->clear();
	}
}

void ofApp::findSphere(recon::CloudPtr src, pcl::PointIndices::Ptr inliers, Eigen::VectorXf& sphereParam)
{
	if (src->size() > 0)
	{
		sphere_model_.reset(new pcl::SampleConsensusModelSphere<recon::PointType>(src));
		sphere_model_->setRadiusLimits(minR_, maxR_);

		pcl::RandomSampleConsensus<recon::PointType> ransac(sphere_model_);

		ransac.setDistanceThreshold(error_);
		ransac.setMaxIterations(samples_);
		ransac.setProbability(percent_);
		ransac.computeModel();


		// obtain results

		ransac.getInliers(inliers->indices);
		ransac.getModelCoefficients(sphereParam);
	}
}

void ofApp::performICPTransformationEstimation()
{
	// perform icp on calib_position pointclouds
	std::list<recon::AbstractSensor::Ptr>::iterator it;
	std::list<recon::AbstractSensor::Ptr>::iterator ref = sensor_list_.begin();
	for (it = sensor_list_.begin(); it != sensor_list_.end(); ++it)
	{
		// estimate transformation from sensor2 to sensor1
		auto sensor1 = *ref;
		auto sensor2 = *it;
		if (sensor1 != sensor2)
		{
			auto cloud1 = calib_positions_[sensor1->getId()];
			auto cloud2 = calib_positions_[sensor2->getId()];
			if (cloud1->size() >= 3 && cloud2->size() >= 3)
			{
				pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
				icp.setInputCloud(cloud2);
				icp.setInputTarget(cloud1);
				icp.setMaxCorrespondenceDistance(icpDistanceThreshold_ * 1000);
				icp.setMaximumIterations(icpIterations_);
				pcl::PointCloud<pcl::PointXYZ> cloud_reg;
				icp.align(cloud_reg);


				auto trans = Eigen::Affine3f(icp.getFinalTransformation());
				Eigen::Vector4f t(trans.translation().x() / 1000, trans.translation().y() / 1000, trans.translation().z() / 1000, 0);
				Eigen::Quaternionf r = Eigen::Quaternionf(trans.rotation());
				recon::CameraExtrinsics::Ptr ext(new recon::CameraExtrinsics(t, r));

				sensor2->setDepthExtrinsics(ext);
			}
		}
	}
}

void ofApp::loadCalibrationFromFile()
{
	SensorCalibrationSettings set;
	for (auto& s : sensor_list_)
	{
		set.loadCalibration(s, s->getId());
	}
}

void ofApp::saveCalibrationToFile()
{
	SensorCalibrationSettings set;

	for (auto& s : sensor_list_)
	{
		set.saveCalibration(s, s->getId());
	}
}
