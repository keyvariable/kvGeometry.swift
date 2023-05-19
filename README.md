# kvGeometry

![Swift 5.2](https://img.shields.io/badge/swift-5.2-green.svg)
![Linux](https://img.shields.io/badge/os-linux-green.svg)
![macOS](https://img.shields.io/badge/os-macOS-green.svg)
![iOS](https://img.shields.io/badge/os-iOS-green.svg)

A collection of geometry auxiliaries on Swift. For example:

- Basic shapes.
- Basic CSG (Constructive Solid Geometry).
- Numerical tolerance handling.


## Supported Platforms

This package is completely crossplatform.


## Getting Started

### Swift Tools 5.2+

#### Package Dependencies:

```swift
dependencies: [
    .package(url: "https://github.com/keyvariable/kvGeometry.swift.git", from: "0.1.1"),
]
```

#### Target Dependencies:

```swift
dependencies: [
    .product(name: "kvGeometry", package: "kvGeometry.swift"),
    .product(name: "kvCSG", package: "kvGeometry.swift"),
]
```

### Xcode

Documentation: [Adding Package Dependencies to Your App](https://developer.apple.com/documentation/xcode/adding_package_dependencies_to_your_app).


## Authors

- Svyatoslav Popov ([@sdpopov-keyvariable](https://github.com/sdpopov-keyvariable), [sdpopov@gmail.com](mailto:sdpopov@gmail.com)).
