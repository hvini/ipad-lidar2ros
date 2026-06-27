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

import UIKit
import ARKit
import OSLog

final class ViewController: UIViewController, ARSessionDelegate {
    private let logger = Logger(subsystem: "com.christophebedard.lidar2ros", category: "ViewController")
    
    private let helpPageButton = UIButton()
    private var mainView: UIStackView!
    
    private var pubController: PubController!
    private var session: ARSession!
    private var rosControllerViewProvider: RosControllerViewProvider!
    
    public func setPubManager(pubManager: PubManager) {
        self.logger.debug("setPubManager")
        self.pubController = pubManager.pubController
        self.session = pubManager.session
    }
    
    override func viewDidLoad() {
        super.viewDidLoad()
        
        self.logger.debug("viewDidLoad")
        
        session.delegate = self
        view.backgroundColor = .systemGroupedBackground
        
        // Help page/message button
        let helpIcon = UIImage(systemName: "questionmark.circle.fill", withConfiguration: UIImage.SymbolConfiguration(scale: .large))?.withTintColor(UIColor(white: 0.5, alpha: 1.0), renderingMode: .alwaysOriginal)
        self.helpPageButton.setImage(helpIcon, for: .normal)
        self.helpPageButton.addTarget(self, action: #selector(showHelp), for: .touchUpInside)
        self.helpPageButton.translatesAutoresizingMaskIntoConstraints = false
        
        self.rosControllerViewProvider = RosControllerViewProvider(pubController: self.pubController!, session: self.session)
        
        // Then stacked vertically
        self.mainView = UIStackView(arrangedSubviews: [rosControllerViewProvider.view!])
        self.mainView.translatesAutoresizingMaskIntoConstraints = false
        self.mainView.axis = .vertical
        self.mainView.spacing = 20
        
        let scrollView = UIScrollView()
        scrollView.translatesAutoresizingMaskIntoConstraints = false
        scrollView.addSubview(mainView)
        
        view.addSubview(scrollView)
        view.addSubview(helpPageButton)
        
        NSLayoutConstraint.activate([
            scrollView.topAnchor.constraint(equalTo: view.safeAreaLayoutGuide.topAnchor, constant: 20),
            scrollView.bottomAnchor.constraint(equalTo: view.safeAreaLayoutGuide.bottomAnchor, constant: -80),
            scrollView.leadingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.leadingAnchor, constant: 20),
            scrollView.trailingAnchor.constraint(equalTo: view.safeAreaLayoutGuide.trailingAnchor, constant: -20),
            
            mainView.topAnchor.constraint(equalTo: scrollView.contentLayoutGuide.topAnchor),
            mainView.bottomAnchor.constraint(equalTo: scrollView.contentLayoutGuide.bottomAnchor),
            mainView.leadingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.leadingAnchor),
            mainView.trailingAnchor.constraint(equalTo: scrollView.contentLayoutGuide.trailingAnchor),
            mainView.widthAnchor.constraint(equalTo: scrollView.frameLayoutGuide.widthAnchor),
            
            self.helpPageButton.bottomAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.bottomAnchor, constant: -30),
            self.helpPageButton.rightAnchor.constraint(equalTo: self.view.safeAreaLayoutGuide.rightAnchor, constant: -30)
        ])
    }
    
    @objc
    private func showHelp() {
        let helpAlertController = UIAlertController(title: nil, message: nil, preferredStyle: .alert)
        helpAlertController.title = "Help"
        helpAlertController.message = """
This application publishes iPad/iPhone sensor data to a rosbridge using the rosbridge v2.0 protocol.

Launch a rosbridge on a computer accessible from this device through the network. Then set the remote bridge IP and port to point to it.

Change topic names, enable/disable publishing, or change publishing rate.

For more information, see instructions linked below.
"""
        let openInstructionsLinkAction = UIAlertAction(title: "open instructions", style: .default) { (action: UIAlertAction) in
            let url = URLComponents(string: "https://github.com/christophebedard/ipad-lidar2ros#using-the-app")!
            UIApplication.shared.open(url.url!)
        }
        let openIssuesLinkAction = UIAlertAction(title: "submit feature request or report bug", style: .default) { (action: UIAlertAction) in
            let url = URLComponents(string: "https://github.com/christophebedard/ipad-lidar2ros/issues")!
            UIApplication.shared.open(url.url!)
        }
        let closeAction = UIAlertAction(title: "close", style: .cancel)
        helpAlertController.addAction(openInstructionsLinkAction)
        helpAlertController.addAction(openIssuesLinkAction)
        helpAlertController.addAction(closeAction)
        self.present(helpAlertController, animated: true)
    }
    
    override func viewWillAppear(_ animated: Bool) {
        super.viewWillAppear(animated)
        
        // Create a world-tracking configuration, and
        // enable the scene depth frame-semantic.
        let configuration = ARWorldTrackingConfiguration()
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
            configuration.frameSemantics = .sceneDepth
        }

        // Run the view's session
        session.run(configuration)
        
        // The screen shouldn't dim during AR experiences.
        UIApplication.shared.isIdleTimerDisabled = true
    }
    
    func session(_ session: ARSession, didFailWithError error: Error) {
        // Present an error message to the user.
        guard error is ARError else { return }
        let errorWithInfo = error as NSError
        let messages = [
            errorWithInfo.localizedDescription,
            errorWithInfo.localizedFailureReason,
            errorWithInfo.localizedRecoverySuggestion
        ]
        let errorMessage = messages.compactMap({ $0 }).joined(separator: "\n")
        DispatchQueue.main.async {
            // Present an alert informing about the error that has occurred.
            let alertController = UIAlertController(title: "The AR session failed.", message: errorMessage, preferredStyle: .alert)
            let restartAction = UIAlertAction(title: "Restart Session", style: .default) { _ in
                alertController.dismiss(animated: true, completion: nil)
                if let configuration = self.session.configuration {
                    self.session.run(configuration, options: .resetSceneReconstruction)
                }
            }
            alertController.addAction(restartAction)
            self.present(alertController, animated: true, completion: nil)
        }
    }
}
