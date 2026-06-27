// Copyright 2021 Christophe Bedard
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

import Foundation
import ARKit
import OSLog

/// Class that manages and instigates the publishing.
///
/// The actual work is done in a background thread.
final class PubManager {
    private let logger = Logger(subsystem: "com.christophebedard.lidar2ros", category: "PubManager")
    
    public let session = ARSession()
    public let pubController: PubController
    private let interface = RosInterface()
    
    private var pubTf: ControlledStaticPublisher
    // private var pubTfStatic: ControlledStaticPublisher
    private var pubDepth: ControlledPublisher
    private var pubPointCloud: ControlledPublisher
    private var pubCamera: ControlledPublisher
    
    public init() {
        /// Create controlled pub objects for all publishers
        self.pubTf = ControlledStaticPublisher(interface: self.interface, type: tf2_msgs__TFMessage.self, topicName: "/tf")
        // FIXME: using /tf only for now because /tf_static does not seem to work
        // self.pubTfStatic = ControlledStaticPublisher(interface: self.interface, type: tf2_msgs__TFMessage.self, topicName: "/tf")
        self.pubDepth = ControlledPublisher(interface: self.interface, type: sensor_msgs__Image.self)
        self.pubPointCloud = ControlledPublisher(interface: self.interface, type: sensor_msgs__PointCloud2.self)
        self.pubCamera = ControlledPublisher(interface: self.interface, type: sensor_msgs__Image.self)
        
        let controlledPubs: [PubController.PubType: [ControlledPublisher]] = [
            //.transforms: [self.pubTf, self.pubTfStatic],
            .transforms: [self.pubTf],
            .depth: [self.pubDepth],
            .pointCloud: [self.pubPointCloud],
            .camera: [self.pubCamera],
        ]
        self.pubController = PubController(pubs: controlledPubs, interface: self.interface)
    }
    
    private func startPubThread(id: String, pubType: PubController.PubType, publishFunc: @escaping () -> Void) {
        self.logger.debug("start pub thread: \(id)")
        DispatchQueue.global(qos: .background).async {
            Thread.current.name = "PubManager: \(id)"
            var last = Date().timeIntervalSince1970
            while true {
                if !self.pubController.isEnabled {
                    continue
                }
                let interval = 1.0 / self.pubController.getPubRate(pubType)!
                // TODO find a better way: seems like busy sleep is the
                // most reliable way to do this but it wastes CPU time
                var now = Date().timeIntervalSince1970
                while now - last < interval {
                    now = Date().timeIntervalSince1970
                }
                last = Date().timeIntervalSince1970
                publishFunc()
            }
        }
    }
    
    /// Start managed publishing.
    public func start() {
        self.logger.debug("start")
        self.startPubThread(id: "tf", pubType: .transforms, publishFunc: self.publishTf)
        self.startPubThread(id: "depth", pubType: .depth, publishFunc: self.publishDepth)
        self.startPubThread(id: "pointcloud", pubType: .pointCloud, publishFunc: self.publishPointCloud)
        // TODO fix/implement
        // self.startPubThread(id: "camera", pubType: .camera, publishFunc: self.publishCamera)
    }
    
    private func publishTf() {
        guard let currentFrame = self.session.currentFrame else {
            return
        }
        let timestamp = currentFrame.timestamp
        let cameraTf = currentFrame.camera.transform
        // TODO revert when /tf_static works
        // self.pubTf.publish(RosMessagesUtils.tfToTfMsg(time: time, tf: cameraTf))
        // self.pubTfStatic.publish(RosMessagesUtils.getTfStaticMsg(time: time))
        var tfMsg = RosMessagesUtils.tfToTfMsg(time: timestamp, tf: cameraTf)
        let tfStaticMsg = RosMessagesUtils.getTfStaticMsg(time: timestamp)
        tfMsg.transforms.append(contentsOf: tfStaticMsg.transforms)
        self.pubTf.publish(tfMsg)
    }
    
    private func publishDepth() {
        guard let currentFrame = self.session.currentFrame,
              let sceneDepth = currentFrame.smoothedSceneDepth ?? currentFrame.sceneDepth else {
                return
        }
        let depthMap = sceneDepth.depthMap
        let timestamp = currentFrame.timestamp
        self.pubDepth.publish(RosMessagesUtils.depthMapToImage(time: timestamp, depthMap: depthMap))
    }
    
    private func publishPointCloud() {
        guard let currentFrame = self.session.currentFrame,
              let sceneDepth = currentFrame.smoothedSceneDepth ?? currentFrame.sceneDepth else {
                return
        }
        let depthMap = sceneDepth.depthMap
        
        let timestamp = currentFrame.timestamp
        let cameraIntrinsics = currentFrame.camera.intrinsics
        let cameraResolution = currentFrame.camera.imageResolution
        
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        
        let scaleX = Float(width) / Float(cameraResolution.width)
        let scaleY = Float(height) / Float(cameraResolution.height)
        
        let fx = cameraIntrinsics[0][0] * scaleX
        let fy = cameraIntrinsics[1][1] * scaleY
        let cx = cameraIntrinsics[2][0] * scaleX
        let cy = cameraIntrinsics[2][1] * scaleY
        
        CVPixelBufferLockBaseAddress(depthMap, .readOnly)
        
        var confAddress: UnsafeMutablePointer<UInt8>? = nil
        var confRowStride = 0
        if let confMap = sceneDepth.confidenceMap {
            CVPixelBufferLockBaseAddress(confMap, .readOnly)
            confAddress = CVPixelBufferGetBaseAddressOfPlane(confMap, 0)?.assumingMemoryBound(to: UInt8.self) ?? CVPixelBufferGetBaseAddress(confMap)?.assumingMemoryBound(to: UInt8.self)
            confRowStride = CVPixelBufferGetBytesPerRow(confMap)
        }
        
        defer {
            if let confMap = sceneDepth.confidenceMap {
                CVPixelBufferUnlockBaseAddress(confMap, .readOnly)
            }
            CVPixelBufferUnlockBaseAddress(depthMap, .readOnly)
        }
        
        var baseAddress = CVPixelBufferGetBaseAddressOfPlane(depthMap, 0)
        if baseAddress == nil {
            baseAddress = CVPixelBufferGetBaseAddress(depthMap)
        }
        
        var points: [ColoredPoint] = []
        
        let cameraImage = currentFrame.capturedImage
        CVPixelBufferLockBaseAddress(cameraImage, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(cameraImage, .readOnly) }
        
        let yPlane = CVPixelBufferGetBaseAddressOfPlane(cameraImage, 0)!.assumingMemoryBound(to: UInt8.self)
        let cbCrPlane = CVPixelBufferGetBaseAddressOfPlane(cameraImage, 1)!.assumingMemoryBound(to: UInt8.self)
        let yBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(cameraImage, 0)
        let cBytesPerRow = CVPixelBufferGetBytesPerRowOfPlane(cameraImage, 1)
        let camW = Float(CVPixelBufferGetWidth(cameraImage))
        let camH = Float(CVPixelBufferGetHeight(cameraImage))
        let scaleCamX = camW / Float(width)
        let scaleCamY = camH / Float(height)
        if let baseAddress = baseAddress {
            let floatBuffer = baseAddress.assumingMemoryBound(to: Float32.self)
            let bytesPerRow = CVPixelBufferIsPlanar(depthMap) ? CVPixelBufferGetBytesPerRowOfPlane(depthMap, 0) : CVPixelBufferGetBytesPerRow(depthMap)
            let rowStride = bytesPerRow / MemoryLayout<Float32>.stride
            
            let step = 1
            points.reserveCapacity((width / step) * (height / step))
            for y in Swift.stride(from: 0, to: height, by: step) {
                for x in Swift.stride(from: 0, to: width, by: step) {
                    if let confBuf = confAddress {
                        // Keep medium (1) and high (2) confidence. Dropping 1 causes objects to vanish.
                        let conf = confBuf[y * confRowStride + x]
                        if conf < 1 { continue }
                    }
                    
                    let depth = floatBuffer[y * rowStride + x]
                    if depth > 0 {
                        let ptX = (Float(x) - cx) * depth / fx
                        let ptY = (Float(y) - cy) * depth / fy
                        let ptZ = -depth
                        
                        // Transform from ARKit landscape camera frame to iPhone Portrait ROS frame
                        // ARKit camera (Landscape Right): X_cam = bottom, Y_cam = right, Z_cam = backward
                        // ROS Portrait device "ipad" frame: X_ipad = forward, Y_ipad = left, Z_ipad = up
                        // Transformation: X_ipad = -Z_cam, Y_ipad = Y_cam, Z_ipad = -X_cam
                        let ipadX = -ptZ
                        let ipadY = ptY
                        let ipadZ = -ptX
                        
                        let cX = min(Int(Float(x) * scaleCamX), Int(camW) - 1)
                        let cY = min(Int(Float(y) * scaleCamY), Int(camH) - 1)
                        let yIdx = cY * yBytesPerRow + cX
                        let cIdx = (cY / 2) * cBytesPerRow + (cX / 2) * 2
                        
                        let yVal = Float(yPlane[yIdx])
                        let cbVal = Float(cbCrPlane[cIdx]) - 128.0
                        let crVal = Float(cbCrPlane[cIdx + 1]) - 128.0
                        
                        var r = yVal + 1.402 * crVal
                        var g = yVal - 0.344136 * cbVal - 0.714136 * crVal
                        var b = yVal + 1.772 * cbVal
                        
                        let rInt = UInt32(max(0, min(255, r)))
                        let gInt = UInt32(max(0, min(255, g)))
                        let bInt = UInt32(max(0, min(255, b)))
                        
                        // Foxglove expects little-endian B, G, R, 0
                        let rgbPacked = (rInt << 16) | (gInt << 8) | bInt
                        
                        points.append(ColoredPoint(x: ipadX, y: ipadY, z: ipadZ, rgb: rgbPacked))
                    }
                }
            }
        }
        
        self.pubPointCloud.publish(RosMessagesUtils.pointsToPointCloud2(time: timestamp, points: points))
    }
    
//    private func publishCamera() {
//        guard let currentFrame = self.session.currentFrame else {
//            return
//        }
//        let timestamp = currentFrame.timestamp
//        let cameraImage = currentFrame.capturedImage
//        self.pubCamera.publish(RosMessagesUtils.pixelBufferToImage(time: time, pixelBuffer: cameraImage))
//    }
}
