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
import UIKit
import OSLog
import ARKit

/// Class providing a view with ROS-related controls.
final class RosControllerViewProvider {
    private let logger = Logger(subsystem: "com.christophebedard.lidar2ros", category: "RosControllerViewProvider")
    
    // Auto-connected URL
    private let hardcodedUrl = "192.168.15.9:9090"
    
    private struct PubEntry {
        var label: UILabel
        var labelText: String
        var topicNameField: UITextField?
        var defaultTopicName: String?
        var stateSwitch: UISwitch
        var rateStepper: UIStepper
        var rateStepperLabel: UILabel
        var rateMin: Double = 0.5
        var rateMax: Double = 30.0
        var rateDefault: Double = PubController.defaultRate
        var rateStep: Double = 0.5
    }
    
    private var pubEntries: [PubController.PubType: PubEntry]! = nil
    private var transformsEntry: PubEntry! = nil
    private var depthEntry: PubEntry! = nil
    private var pointCloudEntry: PubEntry! = nil
    private var cameraEntry: PubEntry! = nil
    
    private let session: ARSession
    private let pubController: PubController
    
    /// The provided view.
    public private(set) var view: UIView! = nil
    
    public init(pubController: PubController, session: ARSession) {
        self.logger.debug("init")
        
        self.pubController = pubController
        self.session = session
        
        // Pub UI entries
        self.transformsEntry = self.createPubEntry(pubType: .transforms, labelText: "Transforms")
        self.depthEntry = self.createPubEntry(pubType: .depth, labelText: "Depth map", defaultTopicName: "/ipad/depth")
        self.pointCloudEntry = self.createPubEntry(pubType: .pointCloud, labelText: "Point cloud", defaultTopicName: "/ipad/pointcloud")
        self.cameraEntry = self.createPubEntry(pubType: .camera, labelText: "Camera", defaultTopicName: "/ipad/camera")
        
        self.pubEntries = [
            .transforms: self.transformsEntry,
            .depth: self.depthEntry,
            .pointCloud: self.pointCloudEntry,
            .camera: self.cameraEntry,
        ]
        self.pubEntries.forEach { (_: PubController.PubType, pubEntry: PubEntry) in
            self.initViewsFromEntry(pubEntry)
        }
        
        // --- Premium UI ---
        let titleLabel = UILabel()
        titleLabel.text = "Sensor Streams"
        titleLabel.font = UIFont.systemFont(ofSize: 28, weight: .bold)
        
        let rosStackView = UIStackView(arrangedSubviews: [
            titleLabel,
            self.createEntryView(pubEntry: self.transformsEntry),
            self.createEntryView(pubEntry: self.depthEntry),
            self.createEntryView(pubEntry: self.pointCloudEntry),
            self.createEntryView(pubEntry: self.cameraEntry)
        ])
        rosStackView.translatesAutoresizingMaskIntoConstraints = false
        rosStackView.axis = .vertical
        rosStackView.spacing = 20
        
        self.view = rosStackView
        
        // Auto-connect
        _ = self.pubController.enable(url: self.hardcodedUrl)
    }
    
    private func createPubEntry(pubType: PubController.PubType, labelText: String, defaultTopicName: String? = nil) -> PubEntry {
        let rateDefault = self.pubController.getPubRate(pubType) ?? PubController.defaultRate
        return PubEntry(label: UILabel(), labelText: labelText, topicNameField: nil != defaultTopicName ? UITextField() : nil, defaultTopicName: defaultTopicName, stateSwitch: UISwitch(), rateStepper: UIStepper(), rateStepperLabel: UILabel(), rateDefault: rateDefault)
    }
    
    private func initViewsFromEntry(_ pubEntry: PubEntry) {
        self.initViews(uiLabel: pubEntry.label, labelText: pubEntry.labelText, uiTextField: pubEntry.topicNameField, uiStatusSwitch: pubEntry.stateSwitch, textFieldPlaceholder: pubEntry.defaultTopicName, useAsDefaultText: true, pubEntry: pubEntry)
    }
    
    private func initViews(uiLabel: UILabel, labelText: String, uiTextField: UITextField?, uiStatusSwitch: UISwitch, textFieldPlaceholder: String?, useAsDefaultText: Bool = false, pubEntry: PubEntry? = nil) {
        if nil != uiTextField {
            uiTextField!.borderStyle = UITextField.BorderStyle.bezel
            uiTextField!.clearButtonMode = UITextField.ViewMode.whileEditing
            uiTextField!.autocorrectionType = UITextAutocorrectionType.no
            if nil != textFieldPlaceholder {
                uiTextField!.placeholder = textFieldPlaceholder
                if useAsDefaultText {
                    uiTextField!.text = textFieldPlaceholder
                }
            }
            uiTextField!.addTarget(self, action: #selector(textFieldValueChanged), for: .editingDidEndOnExit)
        }
        uiLabel.attributedText = NSAttributedString(string: labelText)
        uiStatusSwitch.preferredStyle = UISwitch.Style.checkbox
        uiStatusSwitch.addTarget(self, action: #selector(switchStatusChanged), for: .valueChanged)
        if nil != pubEntry {
            pubEntry!.rateStepper.autorepeat = true
            pubEntry!.rateStepper.isContinuous = true
            pubEntry!.rateStepper.minimumValue = pubEntry!.rateMin
            pubEntry!.rateStepper.maximumValue = pubEntry!.rateMax
            pubEntry!.rateStepper.stepValue = pubEntry!.rateStep
            pubEntry!.rateStepper.value = pubEntry!.rateDefault
            pubEntry!.rateStepper.isEnabled = false
            pubEntry!.rateStepper.addTarget(self, action: #selector(stepperValueChanged), for: .valueChanged)
            pubEntry!.rateStepperLabel.text = RosControllerViewProvider.rateAsString(pubEntry!.rateStepper.value)
        }
    }
    
    private func createEntryView(pubEntry: PubEntry) -> UIView {
        let cardView = UIView()
        cardView.backgroundColor = .secondarySystemGroupedBackground
        cardView.layer.cornerRadius = 14
        cardView.layer.shadowColor = UIColor.black.cgColor
        cardView.layer.shadowOpacity = 0.1
        cardView.layer.shadowOffset = CGSize(width: 0, height: 2)
        cardView.layer.shadowRadius = 4
        
        pubEntry.label.font = UIFont.systemFont(ofSize: 18, weight: .semibold)
        
        let headerStack = UIStackView(arrangedSubviews: [pubEntry.label, pubEntry.stateSwitch])
        headerStack.axis = .horizontal
        headerStack.distribution = .equalSpacing
        
        let mainStack = UIStackView(arrangedSubviews: [headerStack])
        mainStack.axis = .vertical
        mainStack.spacing = 12
        mainStack.translatesAutoresizingMaskIntoConstraints = false
        
        if let textField = pubEntry.topicNameField {
            textField.borderStyle = .roundedRect
            textField.backgroundColor = .tertiarySystemGroupedBackground
            mainStack.addArrangedSubview(textField)
        }
        
        let rateStack = UIStackView(arrangedSubviews: [pubEntry.rateStepper, UIView(), pubEntry.rateStepperLabel])
        rateStack.axis = .horizontal
        rateStack.spacing = 10
        mainStack.addArrangedSubview(rateStack)
        
        cardView.addSubview(mainStack)
        NSLayoutConstraint.activate([
            mainStack.topAnchor.constraint(equalTo: cardView.topAnchor, constant: 16),
            mainStack.bottomAnchor.constraint(equalTo: cardView.bottomAnchor, constant: -16),
            mainStack.leadingAnchor.constraint(equalTo: cardView.leadingAnchor, constant: 16),
            mainStack.trailingAnchor.constraint(equalTo: cardView.trailingAnchor, constant: -16)
        ])
        
        return cardView
    }
    
    @objc
    private func textFieldValueChanged(view: UIView) {
        switch view {
        case self.depthEntry.topicNameField:
            self.updatePubTopic(.depth)
        case self.pointCloudEntry.topicNameField:
            self.updatePubTopic(.pointCloud)
        case self.cameraEntry.topicNameField:
            self.updatePubTopic(.camera)
        default:
            break
        }
    }
    
    @objc
    private func switchStatusChanged(view: UIView) {
        switch view {
        case self.transformsEntry.stateSwitch:
            self.updateTopicState(.transforms)
        case self.depthEntry.stateSwitch:
            self.updateTopicState(.depth)
        case self.pointCloudEntry.stateSwitch:
            self.updateTopicState(.pointCloud)
        case self.cameraEntry.stateSwitch:
            self.updateTopicState(.camera)
        default:
            break
        }
    }
    
     @objc
     private func stepperValueChanged(view: UIView) {
        switch view {
        case self.transformsEntry.rateStepper:
            self.updateRate(.transforms)
        case self.depthEntry.rateStepper:
            self.updateRate(.depth)
        case self.pointCloudEntry.rateStepper:
            self.updateRate(.pointCloud)
        case self.cameraEntry.rateStepper:
            self.updateRate(.camera)
        default:
            break
        }
     }
    

    
    private func updatePubTopic(_ pubType: PubController.PubType) {
        let pubEntry = self.pubEntries[pubType]!
        if self.pubController.updatePubTopic(pubType: .depth, topicName: pubEntry.topicNameField?.text!) {
            pubEntry.stateSwitch.setOn(true, animated: true)
            self.updateTopicState(pubType)
        } else {
            // Disable pub and turn off switch
            self.pubController.disablePub(pubType: pubType)
            pubEntry.stateSwitch.setOn(false, animated: true)
        }
    }
    
    private func updateTopicState(_ pubType: PubController.PubType) {
        let pubEntry = self.pubEntries[pubType]!
        if pubEntry.stateSwitch.isOn {
            // Enable publishing
            if self.pubController.enablePub(pubType: pubType, topicName: pubEntry.topicNameField?.text!) {
                pubEntry.rateStepper.isEnabled = true
            } else {
                // Enabling failed, so disable publishing & stepper and turn off switch
                pubEntry.stateSwitch.setOn(false, animated: true)
                pubEntry.rateStepper.isEnabled = false
                self.pubController.disablePub(pubType: pubType)
            }
        } else {
            // Disable publishing and stepper
            self.pubController.disablePub(pubType: pubType)
            pubEntry.rateStepper.isEnabled = false
        }
    }
    
    private func updateRate(_ pubType: PubController.PubType) {
        // Update display
        let pubEntry = self.pubEntries[pubType]!
        let rate = pubEntry.rateStepper.value
        pubEntry.rateStepperLabel.text = RosControllerViewProvider.rateAsString(rate)
        // Update pub rate
        self.pubController.updatePubRate(pubType: pubType, rate: rate)
    }
    
    private static func rateAsString(_ rate: Double) -> String {
        return String(format: "%.1f Hz", rate)
    }
}
