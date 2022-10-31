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
//  KvCsgPolygon3.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 02.10.2022.
//

import kvKit
import simd

import kvGeometry



/// Implementation of a convex polygon in 3D coordinate space as a 2D convex polygon in XY plane and the transformation.
public struct KvCsgPolygon3<Vertex : KvVertex3Protocol, Payload> {

    public typealias Math = Vertex.Math

    public typealias Polygon2 = KvCsgPolygon2<Vertex, Payload>

    public typealias Transform = KvCsgTransform<Math>



    /// Polygon representation in the local coordinate space.
    public var local: Polygon2
    /// Transformation from the local coordinate space to the world coordinate space.
    public var worldTransform: Transform



    /// Memberwise initializer.
    @inlinable
    public init(local: Polygon2, worldTransform: Transform) {
        self.local = local
        self.worldTransform = worldTransform
    }


    /// Initializes a polygon from vertices in 3D coordinate space.
    public init?<Vertices>(_ vertices: Vertices, payload: Payload)
    where Vertices : Sequence, Vertices.Element == Vertex
    {
        guard let worldTransform = KvCsgPolygon3.worldTransform(for: vertices)
        else { return nil }

        let invWorldTransform = worldTransform.inverse

        guard let local = Polygon2.Shape(vertices.lazy.map { Polygon2.Vertex2(invWorldTransform * $0) })
            .map({ Polygon2(shape: $0, payload: payload) })
        else { return nil }

        self.init(local: local, worldTransform: worldTransform)
    }



    // MARK: Operations

    @inlinable public var payload: Payload {
        get { local.payload }
        set { local.payload = newValue }
    }

    /// A boolean value indicating whether the receiver's vertices are in CCW direction relative to the receiver's normal (Y axis).
    @inlinable public var isCCW: Bool { local.isCCW }

    /// Collection of the receiver's vertices.
    @inlinable public var vertices: AnyCollection<Vertex> {
        .init(local._shape.vertices
            .lazy.map { worldTransform.direct * $0.underlying })
    }


    /// The receiver's normal in the world coordinate space.
    @inlinable public var worldFaceNormal: Math.Vector3 { worldTransform.direct.act(normal: .unitZ) * (isCCW ? 1 : -1) }


    /// Invokes *body* with coordinates of the receiver's triangles.
    @inlinable
    public func forEachTriangle(_ body: (Vertex, Vertex, Vertex) -> Void) {
        var iterator = local._shape.vertices
            .lazy.map { worldTransform.direct * $0.underlying }
            .makeIterator()

        guard let first = iterator.next(),
              var prev = iterator.next()
        else { return }

        while let next = iterator.next() {
            body(first, prev, next)
            prev = next
        }
    }


    /// Invokes *body* with vertex indices of the receiver's triangles.
    @inlinable
    public func forEachTripleOfIndices(_ body: (Int, Int, Int) -> Void) {
        let count = local._shape.vertices.count

        guard count >= 3 else { return }

        var prev = 1
        (2 ..< count).forEach { next in
            defer { prev = next }

            body(0, prev, next)
        }
    }


    /// - Returns: Copy of the receiver.
    @inlinable public func clone() -> Self { Self(local: local.clone(), worldTransform: worldTransform) }


    /// Flips the receiver.
    @inlinable public mutating func flip() { local.flip() }


    /// - Returns: Flipped copy of the receiver.
    @inlinable public func flipped() -> Self { Self(local: local.flipped(), worldTransform: worldTransform) }


    /// - Note: If the result is *nil* then polygon is on the splitting plane.
    internal typealias SplitResult = (front: Self?, back: Self?)?


    /// Splits the reciver by z = 0 plane represented by given transformation *t*.
    ///
    /// - Parameter t: Transformation of z = 0 plane to the world coordinate space.
    ///
    /// - Returns: The split result. Both *front* and *back* are *nil* when the receiver is on the z = 0 plane.
    internal func split(by t: Transform) -> SplitResult {
        switch worldTransform.localPlaneIntersection(withPlaneFrom: t) {
        case .some(let line):
            let polygons = local.split(by: line)

            return (front: polygons.front.map { Self(local: $0, worldTransform: worldTransform) },
                    back: polygons.back.map { Self(local: $0, worldTransform: worldTransform) })

        case .none:
            /// A coordinate on the receiver's plane in the planes's local coordinate space.
            let c = t.inverse.act(coordinate: worldTransform.direct.translation)

            var isNegative = false
            if KvIsPositive(c.z, alsoIsNegative: &isNegative) {
                return (front: self, back: nil)
            }
            else if isNegative {
                return (front: nil, back: self)
            }
            else { return nil }
        }
    }


    /// Applies given transformation to the receiver.
    @inlinable public mutating func apply(_ t: KvTransform3<Math>) {
        let (t, scale) = Transform.from(t)

        worldTransform = t * worldTransform

        if let scale = scale {
            local.apply(scale)
        }
    }



    // MARK: Operators

    /// - Returns: Transformed deep copy of the receiver.
    @inlinable public static func *(lhs: KvTransform3<Math>, rhs: Self) -> Self {
        let (t, scale) = Transform.from(lhs)

        return Self(local: scale.map { $0 * rhs.local } ?? rhs.local, worldTransform: t * rhs.worldTransform)
    }



    // MARK: Auxiliaries

    /// Plane in local coordinate space (XY) containig the receiver's vertices.
    @inlinable
    public static var localPlane: KvPlane3<Math> { [ 0, 0, 1, 0 ] }


    /// - Returns: World transformation for given vertices.
    public static func worldTransform<Vertices>(for vertices: Vertices) -> Transform?
    where Vertices : Sequence, Vertices.Element == Vertex
    {
        var iterator = vertices
            .map { $0.coordinate }
            .makeIterator()

        guard let first = iterator.next() else { return nil }

        var v1: Math.Vector3
    init_v1:
        do {
            while let v = iterator.next().map({ $0 - first }) {
                if Math.isNonzero(v) {
                    v1 = v
                    break init_v1
                }
            }

            return nil
        }

        while let v2 = iterator.next().map({ $0 - first }) {
            if let normal = Math.safeNormalize(Math.cross(v1, v2)) {
                return Transform(from: KvPlane3<Math>(normal: normal, at: first))
            }

            v1 = v2
        }

        return nil
    }
    
}



// MARK: where Payload == Void

extension KvCsgPolygon3 where Payload == Void {

    /// Initializes a polygon from vertices in 3D coordinate space.
    @inlinable
    public init?<Vertices>(_ vertices: Vertices, worldTransform: Transform? = nil)
    where Vertices : Sequence, Vertices.Element == Vertex {
        self.init(vertices, payload: ())
    }

}
