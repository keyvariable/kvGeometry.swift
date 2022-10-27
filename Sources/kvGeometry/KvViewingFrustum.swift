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
//
//  KvViewingFrustum.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 25.09.2022.
//

import kvKit



/// Implementation of viewing frustum in 3D coordinate space.
public struct KvViewingFrustum<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Coordinate = Math.Vector3
    public typealias Matrix = KvTransform3<Math>.Matrix
    public typealias Plane = KvPlane3<Math>



    public let left, right, bottom, top, near, far: Plane



    /// Memberwise initializer.
    @inlinable
    public init(left: Plane, right: Plane, bottom: Plane, top: Plane, near: Plane, far: Plane) {
        self.left = left
        self.right = right
        self.bottom = bottom
        self.top = top
        self.near = near
        self.far = far
    }


    /// Initializes a frustum with a perspective projection matrix.
    @inlinable
    public init?(_ projectionMatrix: Matrix) {
        let m = projectionMatrix.transpose

        left = Plane(abcd: m[3] + m[0])
        guard !left.isDegenerate else { return nil }

        right = Plane(abcd: m[3] - m[0])
        guard !right.isDegenerate else { return nil }

        bottom = Plane(abcd: m[3] + m[1])
        guard !bottom.isDegenerate else { return nil }

        top = Plane(abcd: m[3] - m[1])
        guard !top.isDegenerate else { return nil }

        near = Plane(abcd: m[3] + m[2])
        guard !near.isDegenerate else { return nil }

        far = Plane(abcd: m[3] - m[2])
        guard !far.isDegenerate else { return nil }
    }


    /// Initializes a frustum with a perspective projection matrix overriding Z planes.
    @inlinable
    public init?(_ projectionMatrix: Matrix, zNear: Scalar, zFar: Scalar) {
        let m = projectionMatrix.transpose

        left = Plane(abcd: m[3] + m[0])
        guard !left.isDegenerate else { return nil }

        right = Plane(abcd: m[3] - m[0])
        guard !right.isDegenerate else { return nil }

        bottom = Plane(abcd: m[3] + m[1])
        guard !bottom.isDegenerate else { return nil }

        top = Plane(abcd: m[3] - m[1])
        guard !top.isDegenerate else { return nil }

        (near, far) = (zFar < zNear
                       ? (Plane(normal: .unitNZ, d:  zNear), Plane(normal: .unitZ, d: -zFar))
                       : (Plane(normal: .unitZ, d: -zNear), Plane(normal: .unitNZ, d:  zFar)))
    }



    // MARK: Auxiliaries

    /// The zero frustum containing zero point only.
    public static var zero: Self { .init(left:   Plane(abcd: [ 1, 0, 0, 0 ]), right: Plane(abcd: [ -1, 0, 0, 0 ]),
                                         bottom: Plane(abcd: [ 0, 1, 0, 0 ]), top:   Plane(abcd: [ 0, -1, 0, 0 ]),
                                         near:   Plane(abcd: [ 0, 0, 1, 0 ]), far:   Plane(abcd: [ 0, 0, -1, 0 ])) }
    /// The null frustum containing nothing.
    public static var null: Self { .init(left:   Plane(abcd: [ 1, 0, 0, -1 ]), right: Plane(abcd: [ -1, 0, 0, -1 ]),
                                         bottom: Plane(abcd: [ 0, 1, 0, -1 ]), top:   Plane(abcd: [ 0, -1, 0, -1 ]),
                                         near:   Plane(abcd: [ 0, 0, 1, -1 ]), far:   Plane(abcd: [ 0, 0, -1, -1 ])) }
    /// Frustum containing all the space.
    public static var infinite: Self { .init(left:   Plane(abcd: [ 1, 0, 0, .infinity ]), right: Plane(abcd: [ -1, 0, 0, .infinity ]),
                                             bottom: Plane(abcd: [ 0, 1, 0, .infinity ]), top:   Plane(abcd: [ 0, -1, 0, .infinity ]),
                                             near:   Plane(abcd: [ 0, 0, 1, .infinity ]), far:   Plane(abcd: [ 0, 0, -1, .infinity ])) }



    // MARK: Operations

    /// - Returns: Minimum of signed offsets to the receiver's planes.
    ///
    /// - Note: The result is positive whether given point is inside the receiver.
    @inlinable
    public func minimumInnerDistance(to c: Coordinate) -> Scalar {
        Swift.min(left.signedOffset(to: c), right.signedOffset(to: c),
                  bottom.signedOffset(to: c), top.signedOffset(to: c),
                  near.signedOffset(to: c), far.signedOffset(to: c))
    }


    /// - Returns: A boolean value indicating whether given coordinate is inside the receiver or on it's boundaries.
    @inlinable public func contains(_ c: Coordinate) -> Bool { KvIsNotNegative(minimumInnerDistance(to: c)) }


    /// - Returns: A boolean value indicating whether given coordinate is contained by an inset frustum.
    @inlinable
    public func contains(_ c: Coordinate, margin: Scalar) -> Bool {
        KvIs(minimumInnerDistance(to: c), greaterThanOrEqualTo: margin)
    }


    /// - Returns: A copy of the receiver where all the planes are normalized.
    @inlinable
    public mutating func normalized() -> Self {
        Self(left: left.normalized(), right: right.normalized(),
             bottom: bottom.normalized(), top: top.normalized(),
             near: near.normalized(), far: far.normalized())
    }


    /// - Returns: A copy of the receiver where all the planes are safely normalized.
    @inlinable
    public mutating func safeNormalized() -> Self? {
        guard let l = left.safeNormalized(),
              let r = right.safeNormalized(),
              let b = bottom.safeNormalized(),
              let t = top.safeNormalized(),
              let n = near.safeNormalized(),
              let f = far.safeNormalized()
        else { return nil }

        return Self(left: l, right: r, bottom: b, top: t, near: n, far: f)
    }


    /// - Returns: A copy of the receiver where planes are shifted by given offset.
    @inlinable
    public func inset(by d: Scalar) -> Self {
        .init(left: left.translated(by: left.normal * d),
              right: right.translated(by: right.normal * d),
              bottom: bottom.translated(by: bottom.normal * d),
              top: top.translated(by: top.normal * d),
              near: near.translated(by: near.normal * d),
              far: far.translated(by: far.normal * d))
    }

}



// MARK: : KvNumericallyEquatable

extension KvViewingFrustum : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: KvViewingFrustum<Math>) -> Bool {
        self.left.isEqual(to: rhs.left)
        && self.right.isEqual(to: rhs.right)
        && self.bottom.isEqual(to: rhs.bottom)
        && self.top.isEqual(to: rhs.top)
        && self.near.isEqual(to: rhs.near)
        && self.far.isEqual(to: rhs.far)
    }

}



// MARK: : Equatable

extension KvViewingFrustum : Equatable { }



// MARK: : Hashable

extension KvViewingFrustum : Hashable { }



#if canImport(SceneKit)

import SceneKit



// MARK: Integration with SceneKit

extension KvViewingFrustum where Math == KvMathFloatScope {

    /// Initializes an instance from projection matrix given as an instance of *SCNMatrix4*.
    @inlinable public init?(_ projectionMatrix: SCNMatrix4) { self.init(simd_float4x4(projectionMatrix)) }


    /// Initializes an instance from projection matrix of given camera.
    @inlinable public init?(_ camera: SCNCamera) { self.init(camera.projectionTransform) }


    /// Initializes an instance from projection matrix of renderers current point of view.
    @inlinable
    public init?(for renderer: SCNSceneRenderer) {
        guard let camera = renderer.pointOfView?.camera else { return nil }

        self.init(camera)
    }

}


extension KvViewingFrustum where Math == KvMathDoubleScope {

    /// Initializes an instance from projection matrix given as an instance of *SCNMatrix4*.
    @inlinable public init?(_ projectionMatrix: SCNMatrix4) { self.init(simd_double4x4(projectionMatrix)) }


    /// Initializes an instance from projection matrix of given camera.
    @inlinable public init?(_ camera: SCNCamera) { self.init(camera.projectionTransform) }


    /// Initializes an instance from projection matrix of renderers current point of view.
    @inlinable
    public init?(for renderer: SCNSceneRenderer) {
        guard let camera = renderer.pointOfView?.camera else { return nil }

        self.init(camera)
    }

}

#endif // canImport(SceneKit)
