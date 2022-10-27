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
//  KvVertex.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 19.09.2022.
//

import simd

import kvKit



// MARK: - KvVertexProtocol

public protocol KvVertexProtocol {

    associatedtype Math : KvMathScope

    associatedtype Transform
    associatedtype AffineTransform


    // MARK: Copying

    /// - Returns: Copy of the receiver.
    ///
    /// - Note: Nontrivial implementation is required when the receiver is an instance of a class type.
    func clone() -> Self

    /// Makes the receiver to be equal to *rhs*.
    ///
    /// - Note: It's used to preserce redundant memory allocations.
    mutating func copy(rhs: Self)


    // MARK: Flip

    /// Flips orientation of the receiver.
    ///
    /// Usually the normal or tangent space have to be inverted.
    mutating func flip()

    /// - Returns: Copy of the receiver having opposite orientation.
    ///
    /// Usually the normal or tangent space have to be inverted.
    func flipped() -> Self


    // MARK: Linear Interpolation

    /// Applies (in place) linear interpolation of the receiver and other vertex.
    mutating func mix(_ rhs: Self, t: Math.Scalar)

    /// - Returns: Linear interpolation of the receiver and other vertex.
    func mixed(_ rhs: Self, t: Math.Scalar) -> Self


    // MARK: Transformations

    /// Transforms the receiver.
    mutating func apply(_ t: Transform)
    /// Transforms the receiver.
    mutating func apply(_ t: AffineTransform)

    static func *(lhs: Transform, rhs: Self) -> Self
    static func *(lhs: AffineTransform, rhs: Self) -> Self

}


// MARK: Default Implementations

extension KvVertexProtocol {

    /// - Returns: Copy of the receiver having opposite orientation.
    ///
    /// Usually the normal or tangent space have to be inverted.
    @inlinable
    public func flipped() -> Self {
        var v = clone()
        v.flip()
        return v
    }


    /// - Returns: Linear interpolation of the receiver and other vertex.
    @inlinable
    public func mixed(_ rhs: Self, t: Math.Scalar) -> Self {
        var v = clone()
        v.mix(rhs, t: t)
        return v
    }

}



// MARK: - KvVertex2Protocol

public protocol KvVertex2Protocol : KvVertexProtocol
where Transform == KvTransform2<Math>, AffineTransform == KvAffineTransform2<Math>
{

    var coordinate: Coordinate { get set }


    // MARK: Operators

    /// - Returns: A copy of the receiver translated by *rhs* vector.
    static func +(lhs: Self, rhs: Coordinate) -> Self
    /// - Returns: A copy of the receiver translated by –*rhs* vector.
    static func -(lhs: Self, rhs: Coordinate) -> Self

    /// Translates the receiver by *rhs* vector.
    static func +=(lhs: inout Self, rhs: Coordinate)
    /// Translates the receiver by –*rhs* vector.
    static func -=(lhs: inout Self, rhs: Coordinate)

}


extension KvVertex2Protocol {

    public typealias Coordinate = Math.Vector2


    // MARK: Operators

    @inlinable
    public static func *(lhs: Transform, rhs: Self) -> Self {
        var v = rhs.clone()
        v.apply(lhs)
        return v
    }

    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        var v = rhs.clone()
        v.apply(lhs)
        return v
    }


    /// - Returns: A copy of the receiver translated by *rhs* vector.
    @inlinable
    public static func +(lhs: Self, rhs: Coordinate) -> Self {
        var v = lhs.clone()
        v += rhs
        return v
    }

    /// - Returns: A copy of the receiver translated by –*rhs* vector.
    @inlinable
    public static func -(lhs: Self, rhs: Coordinate) -> Self {
        var v = lhs.clone()
        v -= rhs
        return v
    }


    /// Translates the receiver by *rhs* vector.
    @inlinable public static func +=(lhs: inout Self, rhs: Coordinate) { lhs.coordinate += rhs }

    /// Translates the receiver by –*rhs* vector.
    @inlinable public static func -=(lhs: inout Self, rhs: Coordinate) { lhs.coordinate -= rhs }

}



// MARK: - KvVertex3Protocol

public protocol KvVertex3Protocol : KvVertexProtocol
where Transform == KvTransform3<Math>, AffineTransform == KvAffineTransform3<Math>
{

    var coordinate: Coordinate { get set }


    // MARK: Operators

    /// - Returns: A copy of the receiver translated by *rhs* vector.
    static func +(lhs: Self, rhs: Coordinate) -> Self
    /// - Returns: A copy of the receiver translated by –*rhs* vector.
    static func -(lhs: Self, rhs: Coordinate) -> Self

    /// Translates the receiver by *rhs* vector.
    static func +=(lhs: inout Self, rhs: Coordinate)
    /// Translates the receiver by –*rhs* vector.
    static func -=(lhs: inout Self, rhs: Coordinate)

}


extension KvVertex3Protocol {

    public typealias Coordinate = Math.Vector3


    // MARK: Operators

    @inlinable
    public static func *(lhs: Transform, rhs: Self) -> Self {
        var v = rhs.clone()
        v.apply(lhs)
        return v
    }

    @inlinable
    public static func *(lhs: AffineTransform, rhs: Self) -> Self {
        var v = rhs.clone()
        v.apply(lhs)
        return v
    }


    /// - Returns: A copy of the receiver translated by *rhs* vector.
    @inlinable
    public static func +(lhs: Self, rhs: Coordinate) -> Self {
        var v = lhs.clone()
        v += rhs
        return v
    }

    /// - Returns: A copy of the receiver translated by –*rhs* vector.
    @inlinable
    public static func -(lhs: Self, rhs: Coordinate) -> Self {
        var v = lhs.clone()
        v -= rhs
        return v
    }


    /// Translates the receiver by *rhs* vector.
    @inlinable public static func +=(lhs: inout Self, rhs: Coordinate) { lhs.coordinate += rhs }

    /// Translates the receiver by –*rhs* vector.
    @inlinable public static func -=(lhs: inout Self, rhs: Coordinate) { lhs.coordinate -= rhs }

}



// MARK: - KvVertex2

/// Implementation of a vertex in 2D coordinate space having coordinate, normal and 2D texture coordinate.
///
/// *Payload* is a type for additional data that is shared when copying. Use *Void* if unused.
public struct KvVertex2<Math : KvMathScope, Payload> : KvVertex2Protocol {

    public typealias Math = Math

    public typealias Vector = Math.Vector2
    public typealias TextureCoordinate = Math.Vector2

    public typealias Transform = KvTransform2<Math>
    public typealias AffineTransform = KvAffineTransform2<Math>


    public var coordinate: Coordinate
    /// A normal vectior.
    ///
    /// - Note: It's not required to be a unit vector.
    public var normal: Vector
    /// A 2 element texture coordinate
    public var tx0: TextureCoordinate

    public var payload: Payload


    /// - Parameter normal: A unit vector.
    @inlinable
    public init(_ coordinate: Coordinate, normal: Vector, tx0: TextureCoordinate = .zero, payload: Payload) {
        self.coordinate = coordinate
        self.normal = normal
        self.tx0 = tx0
        self.payload = payload
    }


    // MARK: Operations

    /// - Returns: Copy of the receiver.
    @inlinable public func clone() -> Self { self }

    /// Makes the receiver to be equal to *rhs*.
    @inlinable public mutating func copy(rhs: Self) { self = rhs }
    

    /// Flip orientation of the receiver. E.g. nornal should be negated.
    @inlinable public mutating func flip() { normal = -normal }


    /// Linearly interpolates in place between the receiver and other vertex.
    @inlinable
    public mutating func mix(_ rhs: Self, t: Math.Scalar) {
        coordinate = Math.mix(coordinate, rhs.coordinate, t: t)
        normal = Math.slerp(normal, rhs.normal, t: t)
        tx0 = Math.mix(tx0, rhs.tx0, t: t)
    }


    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: Transform) {
        coordinate = t.act(coordinate: coordinate)
        normal = t.act(normal: normal)
    }

    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: AffineTransform) {
        coordinate = t.act(coordinate: coordinate)
        normal = t.act(normal: normal)
    }

}


// MARK: Payload == Void

extension KvVertex2 where Payload == Void {

    @inlinable
    public init(_ coordinate: Coordinate, normal: Vector, tx0: TextureCoordinate) {
        self.init(coordinate, normal: normal, tx0: tx0, payload: ())
    }

}


// MARK: : KvNumericallyEquatable

extension KvVertex2 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isEqual(coordinate, rhs.coordinate)
        && Math.isCoDirectional(normal, rhs.normal)
        && Math.isEqual(tx0, rhs.tx0)
    }

}


// MARK: : Equatable

extension KvVertex2 : Equatable {

    @inlinable
    public static func ==(lhs: Self, rhs: Self) -> Bool {
        lhs.coordinate == rhs.coordinate
        && lhs.normal == rhs.normal
        && lhs.tx0 == rhs.tx0
    }

}


// MARK: : Hashable

extension KvVertex2 : Hashable {

    @inlinable
    public func hash(into hasher: inout Hasher) {
        coordinate.hash(into: &hasher)
        normal.hash(into: &hasher)
        tx0.hash(into: &hasher)
    }

}



// MARK: - KvVertex3

/// Implementation of a vertex in 3D coordinate space having coordinate, normal and 2D texture coordinate.
///
/// *Payload* is a type for additional data that is shared when copying. Use *Void* if unused.
public struct KvVertex3<Math : KvMathScope, Payload> : KvVertex3Protocol {

    public typealias Math = Math

    public typealias Vector = Math.Vector3
    public typealias TextureCoordinate = Math.Vector2

    public typealias Transform = KvTransform3<Math>
    public typealias AffineTransform = KvAffineTransform3<Math>


    public var coordinate: Coordinate
    /// A normal vectior.
    ///
    /// - Note: It's not required to be a unit vector.
    public var normal: Vector
    /// A 2 element texture coordinate
    public var tx0: TextureCoordinate

    public var payload: Payload


    @inlinable
    public init(_ coordinate: Coordinate, normal: Vector, tx0: TextureCoordinate = .zero, payload: Payload) {
        self.coordinate = coordinate
        self.normal = normal
        self.tx0 = tx0
        self.payload = payload
    }


    // MARK: Operations

    /// - Returns: Copy of the receiver.
    @inlinable public func clone() -> Self { self }

    /// Makes the receiver to be equal to *rhs*.
    @inlinable public mutating func copy(rhs: Self) { self = rhs }


    /// Flip orientation of the receiver. E.g. nornal should be negated.
    @inlinable public mutating func flip() { normal = -normal }


    /// Linearly interpolates in place between the receiver and other vertex.
    @inlinable
    public mutating func mix(_ rhs: Self, t: Math.Scalar) {
        coordinate = Math.mix(coordinate, rhs.coordinate, t: t)
        normal = Math.slerp(normal, rhs.normal, t: t)
        tx0 = Math.mix(tx0, rhs.tx0, t: t)
    }


    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: Transform) {
        coordinate = t.act(coordinate: coordinate)
        normal = t.act(normal: normal)
    }

    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: AffineTransform) {
        coordinate = t.act(coordinate: coordinate)
        normal = t.act(normal: normal)
    }

}


// MARK: Payload == Void

extension KvVertex3 where Payload == Void {

    @inlinable
    public init(_ coordinate: Coordinate, normal: Vector, tx0: TextureCoordinate) {
        self.init(coordinate, normal: normal, tx0: tx0, payload: ())
    }

}


// MARK: : KvNumericallyEquatable

extension KvVertex3 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable
    public func isEqual(to rhs: Self) -> Bool {
        Math.isEqual(coordinate, rhs.coordinate)
        && Math.isCoDirectional(normal, rhs.normal)
        && Math.isEqual(tx0, rhs.tx0)
    }

}


// MARK: : Equatable

extension KvVertex3 : Equatable {

    @inlinable
    public static func ==(lhs: Self, rhs: Self) -> Bool {
        lhs.coordinate == rhs.coordinate
        && lhs.normal == rhs.normal
        && lhs.tx0 == rhs.tx0
    }

}


// MARK: : Hashable

extension KvVertex3 : Hashable {

    @inlinable
    public func hash(into hasher: inout Hasher) {
        coordinate.hash(into: &hasher)
        normal.hash(into: &hasher)
        tx0.hash(into: &hasher)
    }

}



// MARK: - KvPosition2

/// Implementation of a vertex in 2D coordinate space having only coordinate.
///
/// *Payload* is a type for additional data that is shared when copying. Use *Void* if unused.
public struct KvPosition2<Math : KvMathScope, Payload> : KvVertex2Protocol {

    public typealias Math = Math

    public typealias Vector = Math.Vector2

    public typealias Transform = KvTransform2<Math>
    public typealias AffineTransform = KvAffineTransform2<Math>


    public var coordinate: Coordinate

    public var payload: Payload


    @inlinable
    public init(_ coordinate: Coordinate, payload: Payload) {
        self.coordinate = coordinate
        self.payload = payload
    }


    // MARK: Operations

    /// - Returns: Copy of the receiver.
    @inlinable public func clone() -> Self { self }

    /// Makes the receiver to be equal to *rhs*.
    @inlinable public mutating func copy(rhs: Self) { self = rhs }


    /// Flip orientation of the receiver. E.g. nornal should be negated.
    @inlinable public mutating func flip() { }


    /// Linearly interpolates in place between the receiver and other vertex.
    @inlinable
    public mutating func mix(_ rhs: Self, t: Math.Scalar) {
        coordinate = Math.mix(coordinate, rhs.coordinate, t: t)
    }


    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: Transform) {
        coordinate = t.act(coordinate: coordinate)
    }

    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: AffineTransform) {
        coordinate = t.act(coordinate: coordinate)
    }

}


// MARK: Payload == Void

extension KvPosition2 where Payload == Void {

    @inlinable public init(_ coordinate: Coordinate) { self.init(coordinate, payload: ()) }


    @inlinable public init(_ x: Coordinate.Scalar, _ y: Coordinate.Scalar) { self.init(Coordinate(x: x, y: y)) }

    @inlinable public init(x: Coordinate.Scalar, y: Coordinate.Scalar) { self.init(Coordinate(x: x, y: y)) }


    // MARK: Auxiliaries

    @inlinable public static var zero: Self { Self(.zero) }
    @inlinable public static var one: Self { Self(.one) }

}


// MARK: : ExpressibleByArrayLiteral

extension KvPosition2 : ExpressibleByArrayLiteral where Payload == Void {

    @inlinable
    public init(arrayLiteral elements: Coordinate.Scalar...) {
        self.init(Coordinate(elements[0], elements[1]))
    }

}


// MARK: : KvNumericallyEquatable

extension KvPosition2 : KvNumericallyEquatable, KvNumericallyZeroEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(coordinate, rhs.coordinate) }


    /// - Returns: A boolean value indicating whether all the receiver is numerically equal to the zeros.
    @inlinable public func isZero() -> Bool { Math.isZero(coordinate) }

}


// MARK: : Equatable

extension KvPosition2 : Equatable {

    @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.coordinate == rhs.coordinate }

}


// MARK: : Hashable

extension KvPosition2 : Hashable {

    @inlinable public func hash(into hasher: inout Hasher) { coordinate.hash(into: &hasher) }

}



// MARK: - KvPosition3

/// Implementation of a vertex in 3D coordinate space having only coordinate.
///
/// *Payload* is a type for additional data that is shared when copying. Use *Void* if unused.
public struct KvPosition3<Math : KvMathScope, Payload> : KvVertex3Protocol {

    public typealias Math = Math

    public typealias Transform = KvTransform3<Math>
    public typealias AffineTransform = KvAffineTransform3<Math>


    public var coordinate: Coordinate

    public var payload: Payload


    @inlinable
    public init(_ coordinate: Coordinate, payload: Payload) {
        self.coordinate = coordinate
        self.payload = payload
    }


    // MARK: Operations

    /// - Returns: Copy of the receiver.
    @inlinable public func clone() -> Self { self }

    /// Makes the receiver to be equal to *rhs*.
    @inlinable public mutating func copy(rhs: Self) { self = rhs }


    /// Flip orientation of the receiver. E.g. nornal should be negated.
    @inlinable public mutating func flip() { }


    /// Linearly interpolates in place between the receiver and other vertex.
    @inlinable
    public mutating func mix(_ rhs: Self, t: Math.Scalar) {
        coordinate = Math.mix(coordinate, rhs.coordinate, t: t)
    }


    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: Transform) {
        coordinate = t.act(coordinate: coordinate)
    }

    /// Transforms the receiver.
    @inlinable
    public mutating func apply(_ t: AffineTransform) {
        coordinate = t.act(coordinate: coordinate)
    }

}


// MARK: Payload == Void

extension KvPosition3 where Payload == Void {

    @inlinable public init(_ coordinate: Coordinate) { self.init(coordinate, payload: ()) }


    @inlinable public init(_ x: Coordinate.Scalar, _ y: Coordinate.Scalar, _ z: Coordinate.Scalar) { self.init(Coordinate(x: x, y: y, z: z)) }

    @inlinable public init(x: Coordinate.Scalar, y: Coordinate.Scalar, z: Coordinate.Scalar) { self.init(Coordinate(x: x, y: y, z: z)) }


    // MARK: Auxiliaries

    @inlinable public static var zero: Self { Self(.zero) }
    @inlinable public static var one: Self { Self(.one) }

}


// MARK: : ExpressibleByArrayLiteral

extension KvPosition3 : ExpressibleByArrayLiteral where Payload == Void {

    @inlinable
    public init(arrayLiteral elements: Coordinate.Scalar...) {
        self.init(Coordinate(elements[0], elements[1], elements[2]))
    }

}


// MARK: : KvNumericallyEquatable

extension KvPosition3 : KvNumericallyEquatable, KvNumericallyZeroEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(coordinate, rhs.coordinate) }


    /// - Returns: A boolean value indicating whether all the receiver is numerically equal to the zeros.
    @inlinable public func isZero() -> Bool { Math.isZero(coordinate) }

}


// MARK: : Equatable

extension KvPosition3 : Equatable {

    @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.coordinate == rhs.coordinate }

}


// MARK: : Hashable

extension KvPosition3 : Hashable {

    @inlinable public func hash(into hasher: inout Hasher) { coordinate.hash(into: &hasher) }

}
