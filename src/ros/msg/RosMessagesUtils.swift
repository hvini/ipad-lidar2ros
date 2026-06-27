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

/// Utilities.
extension Float {
    /// Conversion from Float to array of bytes ([UInt8]).
    var bytes: [UInt8] {
        withUnsafeBytes(of: self, Array.init)
    }
}

/// Utilities for dealing with and creating messages.
final class RosMessagesUtils {
    /// Get builtin_interfaces/Time message from time value.
    public static func getTimestamp(_ time: Double) -> builtin_interfaces__Time {
        let sec = Int32(time)
        let nanosec = UInt32((time - Double(sec)) * 1000000000)
        let time = builtin_interfaces__Time(sec: sec, nanosec: nanosec)
        return time
    }
    
    /// Get sensor_msgs/PointCloud2 message from time and points.
    public static func pointsToPointCloud2(time: Double, points: [ColoredPoint]) -> sensor_msgs__PointCloud2 {
        let header = std_msgs__Header(stamp: self.getTimestamp(time), frame_id: "ipad")
        // Unordered point cloud: width * height = count * 1
        let height = UInt32(1)
        let width = UInt32(points.count)
        // Each value takes 4 bytes (float = 32 bits = 4 bytes)
        let fields = [
            sensor_msgs__PointField(name: "x", offset: UInt32(0), datatype: sensor_msgs__PointField.DATATYPE_FLOAT32, count: UInt32(1)),
            sensor_msgs__PointField(name: "y", offset: UInt32(4), datatype: sensor_msgs__PointField.DATATYPE_FLOAT32, count: UInt32(1)),
            sensor_msgs__PointField(name: "z", offset: UInt32(8), datatype: sensor_msgs__PointField.DATATYPE_FLOAT32, count: UInt32(1)),
            sensor_msgs__PointField(name: "rgb", offset: UInt32(12), datatype: sensor_msgs__PointField.DATATYPE_UINT32, count: UInt32(1)),
        ]
        let is_bigendian = false
        // 4 elements (x,y,z,rgb) * 4 bytes per element
        let point_step = UInt32(4 * 4)
        let row_step = width * point_step
        let data = self.flattenColoredPointArray(points)
        let is_dense = false
        return sensor_msgs__PointCloud2(header: header, height: height, width: width, fields: fields, is_bigendian: is_bigendian, point_step: point_step, row_step: row_step, data: data, is_dense: is_dense)
    }
    
    /// Flatten array of ColoredPoint and encode to Base64 string for efficient rosbridge transport.
    private static func flattenColoredPointArray(_ array: [ColoredPoint]) -> String {
        var data = Data(capacity: array.count * 16)
        for pt in array {
            withUnsafeBytes(of: pt.x) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: pt.y) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: pt.z) { data.append(contentsOf: $0) }
            withUnsafeBytes(of: pt.rgb) { data.append(contentsOf: $0) }
        }
        return data.base64EncodedString()
    }
    
    /// Get sensor_msgs/Image message from time and depth map.
    public static func depthMapToImage(time: Double, depthMap: CVPixelBuffer) -> sensor_msgs__Image {
        let header = std_msgs__Header(stamp: self.getTimestamp(time), frame_id: "ipad_camera")
        let width = CVPixelBufferGetWidth(depthMap)
        let height = CVPixelBufferGetHeight(depthMap)
        let encoding = sensor_msgs__Image.MONO8
        let is_bigendian = UInt8(0)
        let bytesPerRow = CVPixelBufferGetBytesPerRow(depthMap)
        let step = bytesPerRow / 4
        let data = self.depthPixelBufferToArray(buffer: depthMap, width: width, height: height, bytesPerRow: bytesPerRow)
        return sensor_msgs__Image(header: header, height: UInt32(height), width: UInt32(width), encoding: encoding, is_bigendian: is_bigendian, step: UInt32(step), data: data)
    }
    
    /// Extract raw array of values from pixel buffer representing depth data and encode as Base64.
    private static func depthPixelBufferToArray(buffer: CVPixelBuffer, width: Int, height: Int, bytesPerRow: Int) -> String {
        // Lock buffer
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        // Unlock buffer upon exiting
        defer {
            CVPixelBufferUnlockBaseAddress(buffer, .readOnly)
        }

        var data = Data(capacity: width * height)
        if let baseAddress = CVPixelBufferGetBaseAddressOfPlane(buffer, 0) {
            let buffer = baseAddress.assumingMemoryBound(to: UInt8.self)
            for y in (0..<height) {
                for x in (0..<width) {
                    let ix = y * bytesPerRow + x * 4
                    data.append(buffer[ix + 2])
                }
            }
        }
        return data.base64EncodedString()
    }
    
//    /// Get sensor_msgs/Image message from time and image.
//    public static func pixelBufferToImage(time: Double, pixelBuffer: CVPixelBuffer) -> sensor_msgs__Image {
//        let header = std_msgs__Header(stamp: self.getTimestamp(time), frame_id: "ipad_camera")
//        let width = CVPixelBufferGetWidth(pixelBuffer)
//        let height = CVPixelBufferGetHeight(pixelBuffer)
//        let encoding = sensor_msgs__Image.RGB8
//        let is_bigendian = UInt8(0)
//        let bytesPerRow = CVPixelBufferGetBytesPerRow(pixelBuffer)
//        let step = bytesPerRow
//        let data = self.imageBufferToArray(buffer: pixelBuffer, width: width, height: height, bytesPerRow: bytesPerRow)
//        return sensor_msgs__Image(header: header, height: UInt32(height), width: UInt32(width), encoding: encoding, is_bigendian: is_bigendian, step: UInt32(step), data: data)
//    }
    
//    /// Extract raw array of values from pixel buffer representing camera image data.
//    private static func imageBufferToArray(buffer: CVPixelBuffer, width: Int, height: Int, bytesPerRow: Int) -> [UInt8] {
//        // Lock buffer
//        CVPixelBufferLockBaseAddress(buffer, .readOnly)
//        // Unlock buffer upon exiting
//        defer {
//            CVPixelBufferUnlockBaseAddress(buffer, .readOnly)
//        }
//
//        var imgArray: [UInt8] = []
//        if let baseAddress = CVPixelBufferGetBaseAddressOfPlane(buffer, 0) {
//            let buffer = baseAddress.assumingMemoryBound(to: UInt8.self)
//            for y in (0..<height) {
//                for x in (0..<width) {
//                    let ix = y * bytesPerRow + x * 4
//                    imgArray.append(buffer[ix + 1])
//                    imgArray.append(buffer[ix + 2])
//                    imgArray.append(buffer[ix + 3])
//                }
//            }
//        }
//        return imgArray
//    }
    
    /// Get TFMessage message from camera tf.
    public static func tfToTfMsg(time: Double, tf: simd_float4x4) -> tf2_msgs__TFMessage {
        let tfCamera = self.transformStampedFromTf(tf, time: time, frame_id: "arkit_ref", child_frame_id: "ipad_camera")
        return tf2_msgs__TFMessage(transforms: [tfCamera])
    }
    
    /// Get static TFMessage message.
    public static func getTfStaticMsg(time: Double) -> tf2_msgs__TFMessage {
        let tfArkitRef = self.transformStampedFromTf(self.arkitReference, time: time, frame_id: "map_ipad", child_frame_id: "arkit_ref")
        let tfIpad = self.transformStampedFromTf(self.arkitReferenceInverse, time: time, frame_id: "ipad_camera", child_frame_id: "ipad")
        return tf2_msgs__TFMessage(transforms: [tfArkitRef, tfIpad])
    }
    
    /// Get TransformStamped message given various parameters.
    private static func transformStampedFromTf(_ tf: simd_float4x4, time: Double, frame_id: String, child_frame_id: String) -> geometry_msgs__TransformStamped {
        let quatf = simd_quaternion(tf)
        let translation = tf.columns.3
        let header = std_msgs__Header(stamp: self.getTimestamp(time), frame_id: frame_id)
        let translationMsg = geometry_msgs__Vector3(x: Float64(translation.x), y: Float64(translation.y), z: Float64(translation.z))
        let roationMsg = geometry_msgs__Quaternion(x: Float64(quatf.vector.x), y: Float64(quatf.vector.y), z: Float64(quatf.vector.z), w: Float64(quatf.vector.w))
        let tfMsg = geometry_msgs__Transform(translation: translationMsg, rotation: roationMsg)
        return geometry_msgs__TransformStamped(header: header, child_frame_id: child_frame_id, transform: tfMsg)
    }
    
    static let arkitReference: simd_float4x4 = getARKitReference()
    static let arkitReferenceInverse: simd_float4x4 = arkitReference.inverse
    
    /// Get transform for ARKit coordinate system in normal ROS coordinate system.
    public static func getARKitReference() -> simd_float4x4 {
        // ARKit Gravity reference wrt normal ROS coordinates:
        //   Y up (-gravity), Z backward (out from screen/front camera)
        // Note here that according to the documentation, Z should point out from the camera,
        // but this does not seem to be the case with the camera used for AR (back camera)
        let rotZ = matrix_float4x4(simd_quaternion(-90.0 * Float.degreesToRadian, Float3(0, 0, 1)))
        let rotY = matrix_float4x4(simd_quaternion(-90.0 * Float.degreesToRadian, Float3(0, 1, 0)))
        return rotY * rotZ
    }
}
