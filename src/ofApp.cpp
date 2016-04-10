#include "ofApp.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
//--------------------------------------------------------------
void ofApp::setup(){
	recon::SensorFactory sensorFac;

	auto nSensors = sensorFac.checkConnectedDevices(true);
	for (int i = 0; i < nSensors; i++) {
		sensor_list_.push_back(sensorFac.createPclOpenNI2Grabber());
		detected_sphere_[sensor_list_.back()->getId()].setResolution(6);
	}




	ui_.setup();
	ui_.add(resolutionSl_.setup(resolution_));
	ui_.add(passMinSl_.setup(passMin_));
	ui_.add(passMaxSl_.setup(passMax_));
	ui_.add(minSl_.setup(min_));
	ui_.add(maxSl_.setup(max_));
	ui_.add(samplesSl_.setup(samples_));
	ui_.add(errorSl_.setup(error_));
	ui_.add(percentSl_.setup(percent_));
	ui_.add(meanSampleSl_.setup(meanSamples_));
}

//--------------------------------------------------------------
void ofApp::update(){
	for (auto &sensor : sensor_list_) {
		auto cloud = sensor->getCloudSource()->getOutputCloud();
		if (cloud != nullptr) {
			// downsample cloud for searching sphere
			recon::CloudPtr cloud_downsampled(new recon::Cloud());
			pcl::VoxelGrid<recon::PointType> sor;
			sor.setInputCloud(cloud);
			sor.setLeafSize(resolution_, resolution_, resolution_);
			sor.filter(*cloud_downsampled);

			// remove background
			recon::CloudPtr cloud_wo_back(new recon::Cloud());
			pcl::PassThrough<recon::PointType> pass;
			pass.setFilterFieldName("z");
			pass.setFilterLimits(passMin_, passMax_);
			pass.setInputCloud(cloud_downsampled);
			pass.filter(*cloud_wo_back);

			// find sphere
			sphere_model_.reset(new pcl::SampleConsensusModelSphere<recon::PointType>(cloud_wo_back));
			sphere_model_->setRadiusLimits(min_, max_);
			pcl::RandomSampleConsensus<recon::PointType> ransac(sphere_model_);
			
			ransac.setDistanceThreshold(error_);
			ransac.setMaxIterations(samples_);
			ransac.setProbability(percent_);
			ransac.computeModel();
			
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
			ransac.getInliers(inliers->indices);




			if (inliers->indices.size() > 0) {
				recon::CloudPtr in_cloud(new recon::Cloud());
				pcl::ExtractIndices<recon::PointType> extract;
				extract.setInputCloud(cloud_wo_back);
				extract.setIndices(inliers);
				extract.setNegative(false);
				extract.filter(*in_cloud);
				createOfMeshFromPoints(in_cloud, ofColor(0, 255, 0, 255), inliers_mesh_[sensor->getId()]);

				recon::CloudPtr out_cloud(new recon::Cloud());
				extract.setNegative(true);
				extract.filter(*out_cloud);
				ofMesh mesh;
				createOfMeshFromPoints(out_cloud, ofColor(255, 255, 255, 64), mesh);
				mesh_map_.erase(sensor->getId());
				mesh_map_.insert(std::pair<int, ofMesh>(sensor->getId(), mesh));

				// get sphere coeffincients
				Eigen::VectorXf s_param;
				ransac.getModelCoefficients(s_param);

				// make ofSpherePrimitive
				sphere_detected_[sensor->getId()] = true;
				meanR_[sensor->getId()] = approxRollingAverage(meanR_[sensor->getId()], s_param[3] * 1000, meanSamples_);
				detected_sphere_[sensor->getId()].setRadius(meanR_[sensor->getId()]);
				
				meanX_[sensor->getId()] = approxRollingAverage(meanX_[sensor->getId()], s_param[0] * 1000, meanSamples_);
				meanY_[sensor->getId()] = approxRollingAverage(meanY_[sensor->getId()], s_param[1] * 1000, meanSamples_);
				meanZ_[sensor->getId()] = approxRollingAverage(meanZ_[sensor->getId()], s_param[2] * 1000, meanSamples_);
				detected_sphere_location_[sensor->getId()].set(meanX_[sensor->getId()], meanY_[sensor->getId()], meanZ_[sensor->getId()]);
			}
			else {
				sphere_detected_[sensor->getId()] = false;
				// make ofMesh for displaying
				ofMesh mesh;
				createOfMeshFromPoints(cloud_wo_back, ofColor(255, 255, 255, 64), mesh);
				mesh_map_.erase(sensor->getId());
				mesh_map_.insert(std::pair<int, ofMesh>(sensor->getId(), mesh));
			}
		}
	}
}

//--------------------------------------------------------------
void ofApp::draw(){
	ofBackground(0);

	ofDrawBitmapString("fps: " + std::to_string(ofGetFrameRate()), 10, 10);

	cam_.begin();
	ofEnableDepthTest();
	for (auto &sensor : sensor_list_) {
		mesh_map_[sensor->getId()].drawVertices();
		if (sphere_detected_[sensor->getId()]) {
			inliers_mesh_[sensor->getId()].draw();
			ofSetColor(255, 0, 0);
			ofPushMatrix();
			ofTranslate(detected_sphere_location_[sensor->getId()]);
			detected_sphere_[sensor->getId()].drawWireframe();
			ofPopMatrix();
		}
	}




	cam_.end();

	ofDisableDepthTest();
	ui_.draw();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}

float ofApp::approxRollingAverage(float avg, float new_sample, int window) {

	avg -= avg / window;
	avg += new_sample / window;

	return avg;
}