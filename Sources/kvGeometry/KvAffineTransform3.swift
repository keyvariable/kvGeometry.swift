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
//  KvAffineTransform3.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 18.11.2022.
//

import kvKit



/// Affine transformation of vectors in 3D coordinate space.
public struct KvAffineTransform3<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Matrix = Math.Matrix3x3
    public typealias NormalMatrix = Math.Matrix3x3

    public typealias Vector = Math.Vector3



    public let matrix: Matrix
    public let inverseMatrix: Matrix

    public let normalMatrix: NormalMatrix



    /// Initializes identity transformation.
    @inlinable
    public init() {
        self.matrix = .identity
        self.inverseMatrix = .identity
        self.normalMatrix = .identity
    }


    @inlinable public init(_ matrix: Matrix) { self.init(matrix, inverseMatrix: matrix.inverse) }

    @inlinable
    public init(_ matrix: Matrix, inverseMatrix: Matrix) {
        self.init(matrix, inverseMatrix, KvAffineTransform3.normalizedScaleComponent(for: inverseMatrix.transpose))
    }

    @usableFromInline
    internal init(_ matrix: Matrix, _ inverseMatrix: Matrix, _ normalMatrix: NormalMatrix) {
        self.matrix = matrix
        self.inverseMatrix = inverseMatrix
        self.normalMatrix = normalMatrix
    }


    /// Initializes a rotation transformation.
    @inlinable
    public init(quaternion: Math.Quaternion) {
        let r = Matrix(quaternion)

        self.init(r, r.transpose, r)
    }


    /// Initializes product of rotation and scale transformations.
    @inlinable
    public init(quaternion: Math.Quaternion, scale: Vector) {
        let scale⁻¹ = 1 / scale

        var m = Matrix(quaternion)
        var m⁻¹ = m.transpose

        m[0] *= scale.x
        m[1] *= scale.y
        m[2] *= scale.z

        m⁻¹[0] *= scale⁻¹
        m⁻¹[1] *= scale⁻¹
        m⁻¹[2] *= scale⁻¹

        self.init(m, inverseMatrix: m⁻¹)
    }

    /// Initializes product of rotation and scale transformations.
    @inlinable public init(quaternion: Math.Quaternion, scale: Scalar) { self.init(quaternion: quaternion, scale: Vector(repeating: scale)) }


    /// Initializes a scale transformation.
    @inlinable
    public init(scale: Vector) {
        let scale⁻¹ = 1 / scale

        self.init(Matrix(diagonal: scale),
                  Matrix(diagonal: scale⁻¹),
                  NormalMatrix(diagonal: KvAffineTransform3.normalizedScaleComponent(for: scale⁻¹)))
    }

    /// Initializes a scale transformation.
    @inlinable public init(scale: Scalar) { self.init(scale: Vector(repeating: scale)) }



    // MARK: Auxiliaries

    @inlinable public static var identity: Self { .init() }


    /// - Returns: Scale component of given tranform matrix.
    ///
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable
    public static func scale(from m: Matrix) -> Vector {
        Vector(x: Math.length(m[0]) * (KvIsNotNegative(m.determinant) ? 1 : -1),
               y: Math.length(m[1]),
               z: Math.length(m[2]))
    }


    /// - Returns: Transformed basis vector for given index.
    ///
    /// See: ``basisX(from:)``, ``basisY(from:)``, ``basisZ(from:)``.
    @inlinable
    public static func basis(_ index: Int, from m: Matrix) -> Vector {
        assert((0..<3).contains(index), "Unvalid basis vector index (\(index)) for KvAffineTransform3")
        return m[index]
    }


    /// - Returns: Transformed X basis vector.
    ///
    /// See: ``basisY(from:)``, ``basisZ(from:)``, ``basisX``, ``basis(_:from:)``.
    @inlinable public static func basisX(from m: Matrix) -> Vector { m[0] }

    /// - Returns: Transformed Y basis vector.
    ///
    /// See: ``basisX(from:)``, ``basisZ(from:)``, ``basisY``, ``basis(_:from:)``.
    @inlinable public static func basisY(from m: Matrix) -> Vector { m[1] }

    /// - Returns: Transformed Z basis vector.
    ///
    /// See: ``basisX(from:)``, ``basisY(from:)``, ``basisZ``, ``basis(_:from:)``.
    @inlinable public static func basisZ(from m: Matrix) -> Vector { m[2] }


    /// - Returns: Scale factor applied to given normal matrix to compensate for the effect on length of normals.
    ///
    /// - Note: This method is to be applied to normal matrices to compensate for the effect on length of normals.
    @inlinable
    public static func normalizedScaleComponent(for m: NormalMatrix) -> NormalMatrix {
        m * Math.rsqrt((Math.length²(m[0]) + Math.length²(m[1]) + Math.length²(m[2])) * ((1.0 as Scalar) / (3.0 as Scalar)))
    }

    /// - Returns: A normalized scale vector.
    ///
    /// - SeeAlso: ``normalizedScaleComponent(for:)-4v9ti``
    @inlinable
    public static func normalizedScaleComponent(for v: Vector) -> Vector {
        v * Math.rsqrt(Math.length²(v) * ((1.0 as Scalar) / (3.0 as Scalar)))
    }


    /// - Returns: Rotation transformation matrix.
    @inlinable
    public static func makeMatrix(quaternion: Math.Quaternion) -> Matrix { Matrix(quaternion) }


    /// - Returns: Rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(quaternion: Math.Quaternion, scale: Vector) -> Matrix {
        var m = Matrix(quaternion)
        m[0] *= scale.x
        m[1] *= scale.y
        m[2] *= scale.z
        return m
    }

    /// - Returns: Rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(quaternion: Math.Quaternion, scale: Scalar) -> Matrix { makeMatrix(quaternion: quaternion, scale: Vector(repeating: scale)) }


    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Vector) -> Matrix { Matrix(diagonal: scale) }

    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Scalar) -> Matrix { makeMatrix(scale: Vector(repeating: scale)) }



    // MARK: Operations

    /// Transformed X basis vector.
    ///
    /// See: ``basisY``, ``basisZ``, ``basisX(from:)``, ``basis(_:)``.
    @inlinable public var basisX: Vector { KvAffineTransform3.basisX(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``basisZ``, ``basisY(from:)``, ``basis(_:)``.
    @inlinable public var basisY: Vector { KvAffineTransform3.basisY(from: matrix) }
    /// Transformed Z basis vector.
    ///
    /// See: ``basisX``, ``basisY``, ``basisZ(from:)``, ``basis(_:)``.
    @inlinable public var basisZ: Vector { KvAffineTransform3.basisZ(from: matrix) }


    /// A boolean value indicating whether the receiver is numerically equal to identity tranformation.
    @inlinable public var isIdentity: Bool { Math.isEqual(matrix, .identity) }


    /// The inverse transform.
    @inlinable public var inverse: Self { Self(inverseMatrix, matrix, KvAffineTransform3.normalizedScaleComponent(for: matrix.transpose)) }


    /// Scale component of the receiver.
    ///
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable public var scale: Vector { KvAffineTransform3.scale(from: matrix) }


    /// - Returns: Transformed basis vector for given index.
    ///
    /// See: ``basisX``, ``basisY``, ``basisZ``, ``basis(_:from:)``.
    @inlinable public func basis(_ index: Int) -> Vector { KvAffineTransform3.basis(index, from: matrix) }


    /// - Returns: Tranformed normal.
    @inlinable public func act(normal n: Vector) -> Vector { normalMatrix * n }

    /// - Returns: Transformed coordinate.
    @inlinable public func act(coordinate c: Vector) -> Vector { act(vector: c) }

    /// - Returns: Transformed vector.
    @inlinable public func act(vector v: Vector) -> Vector { matrix * v }



    // MARK: Operators

    @inlinable
    public static func *(lhs: Self, rhs: Self) -> Self {
        Self(lhs.matrix * rhs.matrix, inverseMatrix: rhs.inverseMatrix * lhs.inverseMatrix)
    }

    @inlinable
    public static func *(lhs: Self, rhs: Vector) -> Vector { lhs.act(vector: rhs) }

}


// MARK: : KvNumericallyEquatable

extension KvAffineTransform3 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(matrix, rhs.matrix) }

}


// MARK: : Equatable

extension KvAffineTransform3 : Equatable {

    @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.matrix == rhs.matrix }

    @inlinable public static func !=(lhs: Self, rhs: Self) -> Bool { lhs.matrix != rhs.matrix }

}
