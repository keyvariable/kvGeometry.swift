// swift-tools-version:5.2
//
//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2021 Svyatoslav Popov.
//
//  Licensed under the Apache License, Version 2.0 (the "License"); you may not use this file except in compliance with
//  the License. You may obtain a copy of the License at
//
//  http://www.apache.org/licenses/LICENSE-2.0
//
//  Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on
//  an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the
//  specific language governing permissions and limitations under the License.
//
//  SPDX-License-Identifier: Apache-2.0
//
//===----------------------------------------------------------------------===//

import PackageDescription

let package = Package(
    name: "kvGeometry.swift",

    // Platform constants are inherited from the dependencies.
    platforms: [ .iOS(.v11), ],

    products: [
        .library(name: "kvGeometry", targets: ["kvGeometry"]),
        .library(name: "kvCSG", targets: ["kvCSG"]),
    ],

    dependencies: [
        .package(url: "https://github.com/keyvariable/kvKit-Swift.git", from: "3.1.0-a.54"),
    ],

    targets: [
        .target(
            name: "kvGeometry",
            dependencies: [ .product(name: "kvKit", package: "kvKit-Swift") ]
        ),
        .target(
            name: "kvCSG",
            dependencies: [ "kvGeometry" ]
        ),
        .testTarget(name: "kvGeometryTests", dependencies: [ "kvGeometry", .product(name: "kvTestKit", package: "kvKit-Swift") ]),
    ]
)
