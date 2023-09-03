# kvGeometry.swift

A cross-platform collection of geometry auxiliaries on Swift. For example:

- Basic shapes.
- Basic CSG (Constructive Solid Geometry).
- Numerical tolerance handling.

Handling of numerical tolerance helps to obtain more stable results.
For example, in the kvCSG package, it helps to reduce the number of extra vertices and polygons in resulting geometry.


## Supported Platforms

This package contains no platform-dependent code.


## Getting Started

#### Package Dependencies:
```swift
.package(url: "https://github.com/keyvariable/kvGeometry.swift.git", from: "0.2.1")
```
#### Target Dependencies:
```swift
.product(name: "kvGeometry", package: "kvGeometry.swift")
.product(name: "kvCSG", package: "kvGeometry.swift")
```
#### Import:
```swift
import kvGeometry
import kvCSG
```


## Authors

- Svyatoslav Popov ([@sdpopov-keyvariable](https://github.com/sdpopov-keyvariable), [sdpopov@gmail.com](mailto:sdpopov@gmail.com)).
