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
//  KvAffineTransform2.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 18.11.2022.
//

import kvKit



/// Affine transformation of vectors in 2D coordinate space.
public struct KvAffineTransform2<Math : KvMathScope> {

    public typealias Math = Math

    public typealias Scalar = Math.Scalar
    public typealias Matrix = Math.Matrix2x2
    public typealias NormalMatrix = Math.Matrix2x2

    public typealias Vector = Math.Vector2



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
        self.init(matrix, inverseMatrix, KvAffineTransform2.normalizedScaleComponent(for: inverseMatrix.transpose))
    }

    @usableFromInline
    internal init(_ matrix: Matrix, _ inverseMatrix: Matrix, _ normalMatrix: NormalMatrix) {
        self.matrix = matrix
        self.inverseMatrix = inverseMatrix
        self.normalMatrix = normalMatrix
    }


    /// Initializes a rotation transformation.
    @inlinable
    public init(angle: Scalar) {
        let r = Matrix(angle: angle)
        let r⁻¹ = r.transpose

        self.init(r, r⁻¹, r)
    }


    /// Initializes product of rotation and scale transformations.
    @inlinable
    public init(angle: Scalar, scale: Vector) {
        let scale⁻¹ = 1 / scale

        var m = Matrix(angle: angle)
        var m⁻¹ = m.transpose

        m[0] *= scale.x
        m[1] *= scale.y

        m⁻¹[0] *= scale⁻¹
        m⁻¹[1] *= scale⁻¹

        self.init(m, inverseMatrix: m⁻¹)
    }

    /// Initializes product of rotation and scale transformations.
    @inlinable public init(angle: Scalar, scale: Scalar) { self.init(angle: angle, scale: Vector(repeating: scale)) }


    /// Initializes a scale transformation.
    @inlinable
    public init(scale: Vector) {
        let scale⁻¹ = 1 / scale

        self.init(Matrix(diagonal: scale),
                  Matrix(diagonal: scale⁻¹),
                  NormalMatrix(diagonal: KvAffineTransform2.normalizedScaleComponent(for: scale⁻¹)))
    }

    /// Initializes a scale transformation.
    @inlinable public init(scale: Scalar) { self.init(scale: Vector(repeating: scale)) }



    // MARK: Auxiliaries

    @inlinable public static var identity: Self { .init() }


    /// - Returns: Scale component of given affine tranform matrix.
    ///
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable
    public static func scale(from m: Matrix) -> Vector {
        Vector(x: Math.length(m[0]) * (KvIsNotNegative(m.determinant) ? 1 : -1),
               y: Math.length(m[1]))
    }


    /// Transformed X basis vector.
    ///
    /// See: ``basisY(from:)``, ``basisX``.
    @inlinable public static func basisX(from m: Matrix) -> Vector { m[0] }

    /// Transformed Y basis vector.
    ///
    /// See: ``basisX(from:)``, ``basisY``.
    @inlinable public static func basisY(from m: Matrix) -> Vector { m[1] }


    /// - Returns: Matrix produced from *m* by normalization of the scale component.
    ///
    /// - Note: This method is to be applied to normal matrices to compensate for the effect on length of normals.
    @inlinable
    public static func normalizedScaleComponent(for m: NormalMatrix) -> NormalMatrix {
        m * Math.rsqrt(0.5 * (Math.length²(m[0]) + Math.length²(m[1])))
    }

    /// - Returns: A normalized scale vector.
    ///
    /// - SeeAlso: ``normalizedScaleComponent(for:)-76mip``
    @inlinable
    public static func normalizedScaleComponent(for v: Vector) -> Vector {
        v * Math.rsqrt(0.5 * Math.length²(v))
    }


    /// - Returns: Rotation transformation matrix.
    @inlinable
    public static func makeMatrix(angle: Scalar) -> Matrix { Matrix(angle: angle) }


    /// - Returns: Rotation-scale transformation matrix.
    @inlinable
    public static func makeMatrix(angle: Scalar, scale: Vector) -> Matrix {
        let (sine, cosine) = Math.sincos(angle)
        return Matrix(Matrix.Column(cosine, sine) * scale.x,
                      Matrix.Column(-sine, cosine) * scale.y)
    }

    /// - Returns: Rotation-scale transformation matrix.
    @inlinable public static func makeMatrix(angle: Scalar, scale: Scalar) -> Matrix { makeMatrix(angle: angle, scale: Vector(repeating: scale)) }


    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Vector) -> Matrix { Matrix(diagonal: scale) }

    /// - Returns: Scale transformation matrix.
    @inlinable public static func makeMatrix(scale: Scalar) -> Matrix { makeMatrix(scale: Vector(repeating: scale)) }



    // MARK: Operations

    /// Transformed X basis vector.
    ///
    /// See: ``basisY``, ``basisX(from:)``.
    @inlinable public var basisX: Vector { KvAffineTransform2.basisX(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``basisY(from:)``.
    @inlinable public var basisY: Vector { KvAffineTransform2.basisY(from: matrix) }


    /// A boolean value indicating whether the receiver is numerically equal to identity tranformation.
    @inlinable public var isIdentity: Bool { Math.isEqual(matrix, .identity) }


    /// The inverse transform.
    @inlinable public var inverse: Self { Self(inverseMatrix, matrix, KvAffineTransform2.normalizedScaleComponent(for: matrix.transpose)) }


    /// Scale component of the receiver.
    ///
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable public var scale: Vector { Self.scale(from: matrix) }


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

extension KvAffineTransform2 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(matrix, rhs.matrix) }

}


// MARK: : Equatable

extension KvAffineTransform2 : Equatable {

    @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.matrix == rhs.matrix }

    @inlinable public static func !=(lhs: Self, rhs: Self) -> Bool { lhs.matrix != rhs.matrix }

}
