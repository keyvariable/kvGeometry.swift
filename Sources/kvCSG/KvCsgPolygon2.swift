//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2022 Svyatoslav Popov (info@keyvar.com).
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
//  KvCsgPolygon2.swift
//  kvCSG
//
//  Created by Svyatoslav Popov on 07.09.2022.
//

import kvKit
import simd

import kvGeometry



/// A wrapper for convex polygons of vertices in the XY plane in 3D coordinate space.
public struct KvCsgPolygon2<Vertex : KvVertex3Protocol, Payload> {

    public typealias Math = Vertex.Math

    public typealias Shape = KvConvex2<Vertex2>



    /// The shape of the receiver as a 2D polygon on XY plane in the local coordinate space.
    @inlinable
    public var shape: Shape {
        get { _shape }
        set { _shape = newValue }
    }

    public var payload: Payload


    /// A boolean value indicating whether vertices of the receiver's shape are in CCW direction.
    @inlinable public var isCCW: Bool { _isCCW }



    /// Memberwise initializer.
    @inlinable
    public init(shape: Shape, payload: Payload) {
        self._shape = shape
        self.payload = payload
        self._isCCW = _shape.isCCW
    }



    @usableFromInline
    internal var _shape: Shape {
        didSet { _isCCW = _shape.isCCW }
    }

    @usableFromInline
    internal var _isCCW: Bool



    // MARK: Operations

    /// - Returns: Copy of the receiver.
    @inlinable public func clone() -> Self { Self(shape: _shape.clone(), payload: payload) }


    /// Reverses the receiver's vertices.
    @inlinable public mutating func reverse() { _shape.reverse() }


    /// Flips the receiver.
    @inlinable public mutating func flip() { _shape.flip() }


    /// Designated to scale polygons.
    @usableFromInline
    internal mutating func apply(_ t: KvTransform3<Math>) {
        _shape = Shape(unsafeVertices: _shape.vertices.map { Vertex2(t * $0.underlying) })
    }

    /// Designated to scale polygons.
    @usableFromInline
    internal mutating func apply(_ t: KvAffineTransform3<Math>) {
        _shape = Shape(unsafeVertices: _shape.vertices.map { Vertex2(t * $0.underlying) })
    }


    /// - Returns: Flipped copy of the receiver.
    @inlinable public func flipped() -> Self { Self(shape: _shape.flipped(), payload: payload) }


    public typealias SplitResult = (front: Self?, back: Self?)


    /// - Returns: Front and back parts of the receiver relative to given line.
    @inlinable
    public func split(by line: KvLine2<Math>) -> SplitResult {
        let result = _shape.split(by: line)

        assert(result.front.map { $0.isCCW == isCCW } ?? true)
        assert(result.back.map { $0.isCCW == isCCW } ?? true)

        return (front: result.front.map { Self(shape: $0, payload: payload) },
                back: result.back.map { Self(shape: $0, payload: payload) })
    }



    // MARK: Operators

    /// Designated to scale polygons.
    @usableFromInline
    internal static func *(lhs: KvTransform3<Math>, rhs: Self) -> Self {
        Self(shape: Shape(unsafeVertices: rhs._shape.vertices.map { Vertex2(lhs * $0.underlying) }), payload: rhs.payload)
    }

    /// Designated to scale polygons.
    @usableFromInline
    internal static func *(lhs: KvAffineTransform3<Math>, rhs: Self) -> Self {
        Self(shape: Shape(unsafeVertices: rhs._shape.vertices.map { Vertex2(lhs * $0.underlying) }), payload: rhs.payload)
    }



    // MARK: .Vertex2

    /// 2D representation of a 3D vertex having *z* = 0.
    public struct Vertex2 : KvVertex2Protocol, KvNumericallyEquatable, KvNumericallyZeroEquatable, Hashable {

        public typealias Math = Vertex.Math

        public typealias Coordinate = Math.Vector2

        public typealias Transform = KvTransform2<Math>
        public typealias AffineTransform = KvAffineTransform2<Math>


        @inlinable
        public var coordinate: Coordinate {
            get { Math.make2(underlying.coordinate) }
            set { (underlying.coordinate.x, underlying.coordinate.y) = (newValue.x, newValue.y) }
        }

        public var underlying: Vertex


        @inlinable
        public init(_ underlying: Vertex) {
            assert(KvIsZero(underlying.coordinate.z), "Vertices of CSG convex polygons must be on the XY plane in the local coordinate space")

            self.underlying = underlying
        }


        // MARK: Operations

        /// - Returns: Copy of the receiver.
        @inlinable public func clone() -> Self { Self(underlying.clone()) }

        /// Makes the receiver to be equal to *rhs*.
        @inlinable public mutating func copy(rhs: Self) { underlying.copy(rhs: rhs.underlying) }


        /// Flip orientation of the receiver. E.g. nornal should be negated.
        @inlinable public mutating func flip() { underlying.flip() }


        /// Linearly interpolates in place between the receiver and other vertex.
        @inlinable public mutating func mix(_ rhs: Self, t: Math.Scalar) { underlying.mix(rhs.underlying, t: t) }


        /// Transforms the receiver.
        @inlinable public mutating func apply(_ t: Transform) {
            coordinate = t.act(coordinate: coordinate)
        }

        /// Transforms the receiver.
        @inlinable
        public mutating func apply(_ t: AffineTransform) {
            coordinate = t.act(coordinate: coordinate)
        }


        // MARK: : KvNumericallyEquatable, KvNumericallyZeroEquatable

        /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
        @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(coordinate, rhs.coordinate) }


        /// - Returns: A boolean value indicating whether all the receiver is numerically equal to the zeros.
        @inlinable public func isZero() -> Bool { Math.isZero(coordinate) }


        // MARK: : Equatable

        @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.coordinate == rhs.coordinate }


        // MARK: : Hashable

        @inlinable public func hash(into hasher: inout Hasher) { coordinate.hash(into: &hasher) }

    }

}



// MARK: Payload == Void

extension KvCsgPolygon2 where Payload == Void {

    @inlinable
    public init(shape: Shape) {
        self.init(shape: shape, payload: ())
    }

}
