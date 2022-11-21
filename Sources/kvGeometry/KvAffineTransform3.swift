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
    /// See: ``basisY``, ``basisZ``, ``Basis/x(from:)``, ``basisVector(at:)``.
    @inlinable public var basisX: Vector { Basis.x(from: matrix) }
    /// Transformed Y basis vector.
    ///
    /// See: ``basisX``, ``basisZ``, ``Basis/y(from:)``, ``basisVector(at:)``.
    @inlinable public var basisY: Vector { Basis.y(from: matrix) }
    /// Transformed Z basis vector.
    ///
    /// See: ``basisX``, ``basisY``, ``Basis/z(from:)``, ``basisVector(at:)``.
    @inlinable public var basisZ: Vector { Basis.z(from: matrix) }


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
    /// See: ``basisX``, ``basisY``, ``basisZ``, ``Basis/vector(at:from:)``.
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

extension KvAffineTransform3 : KvNumericallyEquatable {

    /// - Returns: A boolean value indicating whether the receiver and *rhs* are numerically equal.
    @inlinable public func isEqual(to rhs: Self) -> Bool { Math.isEqual(matrix, rhs.matrix) }

}


// MARK: : Equatable

extension KvAffineTransform3 : Equatable {

    @inlinable public static func ==(lhs: Self, rhs: Self) -> Bool { lhs.matrix == rhs.matrix }

    @inlinable public static func !=(lhs: Self, rhs: Self) -> Bool { lhs.matrix != rhs.matrix }

}



// MARK: .Basis

extension KvAffineTransform3 {

    /// Implementation of basis in 3D coordinate space .
    public struct Basis {

        public typealias Permutation = KvTransform3<Math>.Basis.Permutation



        /// Basis vector.
        public var x, y, z: Vector



        /// Memberwise initializer.
        @inlinable
        public init(x: Vector, y: Vector, z: Vector, origin: Vector = .zero) {
            self.x = x
            self.y = y
            self.z = z
        }


        /// Initializes trivial basis.
        @inlinable public init() { self.init(x: .unitX, y: .unitY, z: .unitZ) }


        /// Initializes basis with vectors in given order.
        @inlinable
        public init(vectors: (Vector, Vector, Vector), in order: Permutation) {
            switch order {
            case .xyz:
                (x, y, z) = vectors
            case .xzy:
                (x, z, y) = vectors
            case .yxz:
                (y, x, z) = vectors
            case .yzx:
                (y, z, x) = vectors
            case .zxy:
                (z, x, y) = vectors
            case .zyx:
                (z, y, x) = vectors
            }
        }


        /// Extracts basis from given transformation matrix.
        @inlinable
        public init(_ m: Matrix) {
            self.init(x: Basis.x(from: m), y: Basis.y(from: m), z: Basis.z(from: m))
        }


        /// Extracts basis from given affine transformation.
        @inlinable public init(_ t: KvAffineTransform3) { self.init(t.matrix) }



        // MARK: Completion

        /// - Returns: A left-handed basis where z = *x* × *y*.
        @inlinable public static func completeLH(x: Vector, y: Vector) -> Basis { Basis(x: x, y: y, z: Math.cross(x, y)) }


        /// - Returns: A left-handed basis where y = *z* × *x*.
        @inlinable public static func completeLH(x: Vector, z: Vector) -> Basis { Basis(x: x, y: Math.cross(z, x), z: z) }


        /// - Returns: A left-handed basis where x = *y* × *z*.
        @inlinable public static func completeLH(y: Vector, z: Vector) -> Basis { Basis(x: Math.cross(y, z), y: y, z: z) }



        // MARK: Access Auxiliaries

        /// Trivial basis.
        @inlinable public static var identity: Basis { Basis() }


        /// - Returns: Transformed basis vector for given *index*.
        ///
        /// See: ``x(from:)``, ``y(from:)``, ``z(from:)``, ``setVector(_:at:in:)``.
        @inlinable
        public static func vector(at index: Int, from m: Matrix) -> Vector {
            assert((0..<3).contains(index), "Invalid basis vector index (\(index)) for KvAffineTransform3")
            return m[index]
        }


        /// Replaces basis vector at given *index* in given *matrix* with given *vector*.
        ///
        /// See: ``setX(in:)``, ``setY(in:)``, ``setZ(in:)``, ``vector(at:from:)``.
        @inlinable
        public static func setVector(_ v: Vector, at index: Int, in m: inout Matrix) {
            assert((0..<3).contains(index), "Invalid basis vector index (\(index)) for KvAffineTransform3")
            m[index] = v
        }


        /// - Returns: Transformed X basis vector.
        ///
        /// See: ``y(from:)``, ``z(from:)``, ``setX(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func x(from m: Matrix) -> Vector { m[0] }


        /// - Returns: Transformed Y basis vector.
        ///
        /// See: ``x(from:)``, ``z(from:)``, ``setY(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func y(from m: Matrix) -> Vector { m[1] }


        /// - Returns: Transformed Z basis vector.
        ///
        /// See: ``x(from:)``, ``y(from:)``, ``setZ(_:in:)``, ``vector(at:from:)``.
        @inlinable public static func z(from m: Matrix) -> Vector { m[2] }


        /// Replaces X basis vector in given *matrix* with given value.
        ///
        /// See: ``setY(_:in:)``, ``setZ(_:in:)``, ``x(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setX(_ v: Vector, in m: inout Matrix) { m[0] = v }


        /// Replaces Y basis vector in given *matrix* with given value.
        ///
        /// See: ``setX(_:in:)``, ``setZ(_:in:)``, ``y(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setY(_ v: Vector, in m: inout Matrix) { m[1] = v }


        /// Replaces Z basis vector in given *matrix* with given value.
        ///
        /// See: ``setX(_:in:)``, ``setY(_:in:)``, ``z(from:)``, ``setVector(_:at:in:)``.
        @inlinable public static func setZ(_ v: Vector, in m: inout Matrix) { m[2] = v }



        // MARK: Orthogonalization Auxiliaries

        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func orthogonalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector) {
            KvTransform3<Math>.Basis.orthogonalized(vectors: v)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process).
        @inlinable
        public static func safeOrthogonalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector)? {
            KvTransform3<Math>.Basis.safeOrthogonalized(vectors: v)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        @inlinable
        public static func orthonormalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector) {
            KvTransform3<Math>.Basis.orthonormalized(vectors: v)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) where each vector is normalized.
        ///            `Nil` is returned if any of vectors becomes degenerate.
        @inlinable
        public static func safeOrthonormalized(vectors v: (Vector, Vector, Vector)) -> (Vector, Vector, Vector)? {
            KvTransform3<Math>.Basis.safeOrthonormalized(vectors: v)
        }


        // MARK: Subscripts

        /// Provides access to the receiver's vectors in given *order*.
        @inlinable
        public subscript(order: Permutation) -> (Vector, Vector, Vector) {
            get {
                switch order {
                case .xyz:
                    return (x, y, z)
                case .xzy:
                    return (x, z, y)
                case .yxz:
                    return (y, x, z)
                case .yzx:
                    return (y, z, x)
                case .zxy:
                    return (z, x, y)
                case .zyx:
                    return (z, y, x)
                }
            }
            set {
                switch order {
                case .xyz:
                    (x, y, z) = newValue
                case .xzy:
                    (x, z, y) = newValue
                case .yxz:
                    (y, x, z) = newValue
                case .yzx:
                    (y, z, x) = newValue
                case .zxy:
                    (z, x, y) = newValue
                case .zyx:
                    (z, y, x) = newValue
                }
            }
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
        @inlinable public var matrix: Matrix { Matrix(x, y, z) }

        /// *KvAffineTransform2* representation of the receiver.
        ///
        /// See: ``matrix``, ``transform``.
        @inlinable public var affineTransform: KvAffineTransform3 { KvAffineTransform3(matrix) }

        /// *KvTransform2* representation of the receiver.
        ///
        /// See: ``affineTransform``.
        @inlinable public var transform: KvTransform3<Math> { KvTransform3(Math.make4(matrix)) }


        /// Normalizes the receiver's vectors.
        ///
        /// See: ``normalized()``.
        @inlinable
        public mutating func normalize() {
            x = Math.normalize(x)
            y = Math.normalize(y)
            z = Math.normalize(z)
        }


        /// - Returns: A copy of the receiver where the vectors are normalized.
        ///
        /// See: ``normalize()``, ``safeNormalized()``.
        @inlinable public func normalized() -> Basis { Basis(x: Math.normalize(x), y: Math.normalize(y), z: Math.normalize(z)) }


        /// - Returns: A copy of the receiver where all the vectors are normalized if nonzero. If any vector is zero then `nil` is returned.
        ///
        /// See: ``normalized()``.
        @inlinable
        public func safeNormalized() -> Basis? {
            guard let x = Math.safeNormalize(x),
                  let y = Math.safeNormalize(y),
                  let z = Math.safeNormalize(z)
            else { return nil }

            return Basis(x: x, y: y, z: z)
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* to the receiver.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public mutating func orthogonalize(order: Permutation = .xyz) {
            _ = { Basis.orthogonalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthogonalize(order:)``, ``safeOrthogonalized(order:)``.
        @inlinable
        public func orthogonalized(order: Permutation = .xyz) -> Basis {
            Basis(vectors: Basis.orthogonalized(vectors: self[order]), in: order)
        }


        /// - Returns: The result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthogonalized(order:)``.
        @inlinable
        public func safeOrthogonalized(order: Permutation = .xyz) -> Basis? {
            Basis.safeOrthogonalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order) }
        }


        /// Applies [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) in given *order* and normalizes to the receiver.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public mutating func orthonormalize(order: Permutation = .xyz) {
            _ = { Basis.orthonormalized(vectors: $0) }(&self[order])
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///
        /// See: ``orthonormalize(order:)``, ``safeOrthonormalized(order:)``.
        @inlinable
        public func orthonormalized(order: Permutation = .xyz) -> Basis {
            Basis(vectors: Basis.orthonormalized(vectors: self[order]), in: order)
        }


        /// - Returns: The normalized result of [modified Gram–Schmidt process](https://en.wikipedia.org/wiki/Gram–Schmidt_process) applied to the receiver's vectors in given *order*.
        ///            `Nil` is returned when zero vector occurs.
        ///
        /// See: ``orthonormalized(order:)``.
        @inlinable
        public func safeOrthonormalized(order: Permutation = .xyz) -> Basis? {
            Basis.safeOrthonormalized(vectors: self[order])
                .map { Basis(vectors: $0, in: order) }
        }

    }

}
