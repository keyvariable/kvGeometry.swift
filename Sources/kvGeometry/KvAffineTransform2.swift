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
    /// See: ``basisY``, ``Basis/x(from:)``, ``basisVector(at:)``.
    @inlinable public var basisX: Vector { Basis.x(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``Basis/y(from:)``, ``basisVector(at:)``.
    @inlinable public var basisY: Vector { Basis.y(from: matrix) }


    /// A boolean value indicating whether the receiver is numerically equal to identity tranformation.
    @inlinable public var isIdentity: Bool { Math.isEqual(matrix, .identity) }


    /// The inverse transform.
    @inlinable public var inverse: Self { Self(inverseMatrix, matrix, KvAffineTransform2.normalizedScaleComponent(for: matrix.transpose)) }


    /// Scale component of the receiver.
    ///
    /// - Note: If determinant of the matrix is negative then X scale element is negative and other elements are non-negative.
    @inlinable public var scale: Vector { Self.scale(from: matrix) }


    /// - Returns: Transformed basis vector for given index.
    ///
    /// See: ``basisX``, ``basisY``, ``Basis/vector(at:from:).
    @inlinable public func basisVector(at index: Int) -> Vector { Basis.vector(at: index, from: matrix) }


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



// MARK: .Basis

extension KvAffineTransform2 {

    /// Implementation of basis in 2D coordinate space.
    public struct Basis {

        public typealias Permutation = KvTransform2<Math>.Basis.Permutation



        /// Basis vector.
        public var x, y: Vector



        /// Memberwise initializer.
        @inlinable
        public init(x: Vector, y: Vector) {
            self.x = x
            self.y = y
        }


        /// Initializes trivial basis.
        @inlinable public init() { self.init(x: .unitX, y: .unitY) }


        /// Initializes basis with vectors in given order.
        @inlinable
        public init(vectors: (Vector, Vector), in order: Permutation) {
            switch order {
            case .xy:
                (x, y) = vectors
            case .yx:
                (y, x) = vectors
            }
        }


        /// Extracts basis from given transformation matrix.
        @inlinable
        public init(_ m: Matrix) {
            self.init(x: Basis.x(from: m), y: Basis.y(from: m))
        }


        /// Extracts basis from given affine transformation.
        @inlinable public init(_ t: KvAffineTransform2<Math>) { self.init(t.matrix) }



        // MARK: Subscripts

        /// Provides access to the receiver's vectors in given *order*.
        @inlinable
        public subscript(order: Permutation) -> (Vector, Vector) {
            get {
                switch order {
                case .xy:
                    return (x, y)
                case .yx:
                    return (y, x)
                }
            }
            set {
                switch order {
                case .xy:
                    (x, y) = newValue
                case .yx:
                    (y, x) = newValue
                }
            }
        }



        // MARK: Completion

        /// - Returns: An orthogonal left-handed basis where Y vector is calculated for given X vector.
        ///
        /// - Note: Y vector has the same length as *x*.
        @inlinable public static func completeLH(x: Vector) -> Basis { Basis(x: x, y: Vector(-x.y, x.x)) }


        /// - Returns: An orthogonal left-handed basis where X vector is calculated for given Y vector.
        ///
        /// - Note: X vector has the same length as *y*.
        @inlinable public static func completeLH(y: Vector) -> Basis { Basis(x: Vector(y.y, -y.x), y: y) }



        // MARK: Access

        /// Trivial basis.
        @inlinable public static var identity: Basis { Basis() }


        /// - Returns: Transformed basis vector for given *index*.
        ///
        /// See: ``x(from:)``, ``y(from:)``, ``setVector(_:at:in:)``.
        @inlinable
        public static func vector(at index: Int, from m: Matrix) -> Vector {
            assert((0..<2).contains(index), "Invalid basis vector index (\(index)) for KvAffineTransform2")
            return m[index]
        }


        /// Replaces basis vector at given *index* in given *matrix* with given *vector*.
        ///
        /// See: ``setX(in:)``, ``setY(in:)``, ``vector(at:from:)``.
        @inlinable
        public static func setVector(_ v: Vector, at index: Int, in m: inout Matrix) {
            assert((0..<2).contains(index), "Invalid basis vector index (\(index)) for KvAffineTransform2")
            m[index] = v
        }


        /// - Returns: Transformed X basis vector.
        ///
        /// See: ``y(from:)``, ``setX(_:in:)``.
        @inlinable public static func x(from m: Matrix) -> Vector { m[0] }


        /// - Returns: Transformed Y basis vector.
        ///
        /// See: ``x(from:)``, ``setY(_:in:)``.
        @inlinable public static func y(from m: Matrix) -> Vector { m[1] }


        /// Replaces X basis vector in given *matrix* with given value.
        ///
        /// See: ``setY(_:in:)``, ``x(from:)``.
        @inlinable public static func setX(_ v: Vector, in m: inout Matrix) { m[0] = v }


        /// Replaces Y basis vector in given *matrix* with given value.
        ///
        /// See: ``setX(_:in:)``, ``y(from:)``.
        @inlinable public static func setY(_ v: Vector, in m: inout Matrix) { m[1] = v }



        // MARK: Orthogonalization

        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func orthogonalized(vectors v: (Vector, Vector)) -> (Vector, Vector) {
            KvTransform2<Math>.Basis.orthogonalized(vectors: v)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func safeOrthogonalized(vectors v: (Vector, Vector)) -> (Vector, Vector)? {
            KvTransform2<Math>.Basis.safeOrthogonalized(vectors: v)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        @inlinable
        public static func orthonormalized(vectors v: (Vector, Vector)) -> (Vector, Vector) {
            KvTransform2<Math>.Basis.orthonormalized(vectors: v)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        ///            `Nil` is returned if any of vectors becomes degenerate.
        @inlinable
        public static func safeOrthonormalized(vectors v: (Vector, Vector)) -> (Vector, Vector)? {
            KvTransform2<Math>.Basis.safeOrthonormalized(vectors: v)
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* to the receiver.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public mutating func orthogonalize(order: Permutation = .xy) {
            _ = { $0 = Basis.orthogonalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthogonalize(order:)``, ``safeOrthogonalized(order:)``.
        @inlinable
        public func orthogonalized(order: Permutation = .xy) -> Basis {
            Basis(vectors: Basis.orthogonalized(vectors: self[order]), in: order)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public func safeOrthogonalized(order: Permutation = .xy) -> Basis? {
            Basis.safeOrthogonalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order) }
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* and normalizes to the receiver.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public mutating func orthonormalize(order: Permutation = .xy) {
            _ = { $0 = Basis.orthonormalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthonormalize(order:)``, ``safeOrthonormalized(order:)``.
        @inlinable
        public func orthonormalized(order: Permutation = .xy) -> Basis {
            Basis(vectors: Basis.orthonormalized(vectors: self[order]), in: order)
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public func safeOrthonormalized(order: Permutation = .xy) -> Basis? {
            Basis.safeOrthonormalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order) }
        }

        

        // MARK: Operations

        /// A boolean value indicating whether the receiver is degenerate.
        @inlinable
        public var isDegenerate: Bool {
            let m = matrix
            return KvIsZero(m.determinant, eps: Math.epsArg(m).tolerance)
        }

        /// A boolean value indicating whether all the receiver's vectors are of unit length.
        @inlinable public var isNormalized: Bool { Math.isUnit(x) && Math.isUnit(y) }

        /// A boolean value indicating wheter the receiiver is orthogonal.
        @inlinable public var isOrthogonal: Bool { Math.isOrthogonal(x, y) }


        /// Matrix representation of the receiver.
        ///
        /// See: ``affineTransform``.
        @inlinable public var matrix: Matrix { Matrix(x, y) }

        /// *KvAffineTransform2* representation of the receiver.
        ///
        /// See: ``matrix``, ``transform``.
        @inlinable public var affineTransform: KvAffineTransform2 { KvAffineTransform2(matrix) }

        /// *KvTransform2* representation of the receiver.
        ///
        /// See: ``affineTransform``.
        @inlinable public var transform: KvTransform2<Math> { KvTransform2(Math.make3(matrix)) }


        /// Normalizes the receiver's vectors.
        ///
        /// See: ``normalized()``.
        @inlinable
        public mutating func normalize() {
            x = Math.normalize(x)
            y = Math.normalize(y)
        }


        /// - Returns: A copy of the receiver where the vectors are normalized.
        ///
        /// See: ``normalize()``, ``safeNormalized()``.
        @inlinable public func normalized() -> Basis { Basis(x: Math.normalize(x), y: Math.normalize(y)) }


        /// - Returns: A copy of the receiver where all the vectors are normalized if nonzero. If any vector is zero then `nil` is returned.
        ///
        /// See: ``normalized()``.
        @inlinable
        public func safeNormalized() -> Basis? {
            guard let x = Math.safeNormalize(x),
                  let y = Math.safeNormalize(y)
            else { return nil }

            return Basis(x: x, y: y)
        }

    }

}
