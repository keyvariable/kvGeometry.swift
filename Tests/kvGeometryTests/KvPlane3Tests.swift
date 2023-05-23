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
//  KvPlane3Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 12.10.2022.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvPlane3Tests : XCTestCase {

    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: Equality Tests

    func testIsEqual() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            typealias P = KvPlane3<Math>

            let cases: [(lhs: P, rhs: P, expected: Bool)] = [
                ([ 0, 1, 0,  0 ], [ 0,  1, 0,  0], true),
                ([ 0, 1, 0,  0 ], [ 0,  2, 0,  0], true),
                ([ 0, 1, 0,  1 ], [ 0,  1, 0,  1], true),
                ([ 0, 1, 0, -1 ], [ 0,  2, 0, -2], true),

                ([ 0, 1, 0,  0 ], [ 0, -1, 0, 0], false),
                ([ 0, 1, 0,  0 ], [ 1,  0, 0, 0], false),
                ([ 0, 1, 0,  0 ], [ 0,  1, 0, 1], false),
                ([ 0, 1, 0, -1 ], [ 0, -1, 0, 1], false),
                ([ 0, 1, 0, -1 ], [ 0, -2, 0, 2], false),
            ]

            cases.forEach { (lhs, rhs, expected) in
                if lhs.isEqual(to: rhs) != expected {
                    XCTFail("lhs.isEqual(to: rhs) != \(expected), where lhs = «\(lhs)», rhs = «\(rhs)»")
                }
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: .contains Coordinate Test

    func testContainsCoordinate() {

        func Run<Math : KvMathScope>(_ math: Math.Type)
        where Math.Scalar.RawSignificand : FixedWidthInteger
        {
            typealias Scalar = Math.Scalar
            typealias P = KvPlane3<Math>

            let angleRange = (0.0 as Scalar) ..< ((2.0 as Scalar) * Scalar.pi)
            let translationRange = (-100.0 as Scalar) ... (100.0 as Scalar)

            (0..<25).forEach { _ in
                let matrix = P.Transform.makeMatrix(
                    translation: Math.random3(in: translationRange),
                    quaternion: .init(angle: .random(in: angleRange), axis: Math.Vector3.unitRandom())
                )

                let plane = { P(normal: $0.z, at: $0.origin) }(P.Transform.Basis(matrix))
                let unitNormal = Math.normalize(plane.normal)


                func Assert(_ tx: Scalar, _ ty: Scalar) {
                    let c_in = P.Transform.act(matrix, coordinate: .init(x: tx, y: ty, z: 0.0 as Scalar))
                    XCTAssert(plane.contains(c_in), "contains: plane = (\(plane)), c = \(c_in), t = (\(tx), \(ty)), distance = \(plane.offset(to: c_in))")

                    let offset: Scalar = max(1.0 as Scalar, Math.abs(c_in).max()) * (1e-3 as Scalar)
                    let c_out = c_in + offset * unitNormal
                    XCTAssert(!plane.contains(c_out), "!contains: plane = (\(plane)), c = \(c_out), t = (\(tx), \(ty)), distance = \(plane.offset(to: c_in))")
                }


                stride(from: -50.0 as Scalar, through: 50.001 as Scalar, by: (0.0 as Scalar).distance(to: 10.0 as Scalar)).forEach { t in
                    Assert(t, 0.0 as Scalar)
                    Assert(0.0 as Scalar, t)
                }

                stride(from: -50.0 as Scalar, through: 50.001 as Scalar, by: (0.0 as Scalar).distance(to: 10.0 as Scalar)).forEach { t in
                    Assert(t, t)
                }
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Plane-plane Intersection Test

    func testIntersectionWithPlane() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            typealias P = KvPlane3<Math>
            typealias L = KvLine3<Math>

            let sqrt1_2 = (0.5 as Math.Scalar).squareRoot()

            let cases: [(lhs: P, rhs: P, expected: L?)] = [
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0,  1, 0 ], at: [ 0,  0,  0 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0, -1, 0 ], at: [ 0,  0,  0 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0,  1, 0 ], at: [ 0,  1,  0 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0, -1, 0 ], at: [ 0,  1,  0 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0,  1, 0 ], at: [ 0, -1,  0 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0, -1, 0 ], at: [ 0, -1,  0 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0,  1, 0 ], at: [ 0,  0,  1 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0, -1, 0 ], at: [ 0,  0,  1 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0,  1, 0 ], at: [ 0,  0, -1 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0, -1, 0 ], at: [ 0,  0, -1 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 1), P(normal: [ 0,  1, 0 ], at: [ 0,  1,  0 ]), nil),
                (P(normal: [ 0, 1, 0 ], d: 1), P(normal: [ 0, -1, 0 ], at: [ 0,  1,  0 ]), nil),

                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0, -sqrt1_2, sqrt1_2 ], d: 0), L(in: [ 1, 0, 0 ], at: .zero)),
                (P(normal: [ 0, 1, 0 ], d: 0), P(normal: [ 0, -sqrt1_2, sqrt1_2 ], d: -sqrt1_2), L(in: [ 1, 0, 0 ], at: [ 0, 0, 1 ])),

                ([ 0, 1, 0, -0.5 ], [ 2, 0, 0, -2 ], L(in: .unitZ, at: [ 1, 0.5, 0 ])),
            ]

            cases.forEach { (lhs, rhs, expected) in
                KvAssertEqual(lhs.intersection(with: rhs), expected)
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }

}
