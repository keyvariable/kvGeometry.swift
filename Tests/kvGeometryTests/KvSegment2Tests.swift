//===----------------------------------------------------------------------===//
//
//  Copyright (c) 2023 Svyatoslav Popov (info@keyvar.com).
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
//  KvSegment2Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 22.05.2023.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvSegment2Tests : XCTestCase {

    typealias V<Math : KvMathScope> = KvPosition2<Math, Void>
    typealias S<Math : KvMathScope> = KvSegment2<V<Math>>



    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: Degenerate Test

    func testContains() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            typealias Scalar = Math.Scalar

            let s = S<Math>(.zero, [ 1, 2 ])
            let step = (small: (0.0 as Scalar).distance(to: 0.05 as Scalar),
                        large: (0.0 as Scalar).distance(to: 0.5 as Scalar) )

            stride(from: 0.0 as Scalar, through: 1.0 as Scalar + 1e-3, by: step.small).forEach { t in
                XCTAssert(s.contains(Math.mix(s.endPoints.0.coordinate, s.endPoints.1.coordinate, t: t)))
            }

            let strides: [StrideTo<Math.Scalar>] = [
                stride(from: -10.0, to: -1.0 - 1e-3, by: step.large),
                stride(from: -1.0, to: -1e-3, by: step.small),
                stride(from: (1.0 as Scalar).advanced(by: step.small), to: 2.0 - 1e-3, by: step.small),
                stride(from: 2.0, to: 10.0 - 1e-3, by: step.large),
            ]

            strides.forEach { stride in
                stride.forEach { t in
                    let c = Math.mix(s.endPoints.0.coordinate, s.endPoints.1.coordinate, t: t)
                    XCTAssertFalse(s.contains(c), "Segment «\(s)» contains unexpected coordinate \(c) corresponding to t = \(t)")
                }
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }


    func testDegenerate() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            let v: V<Math> = [ 1, 2 ]
            let s = S<Math>(v, v)

            XCTAssert(s.contains(v))
            XCTAssert(!s.contains(.zero))
            XCTAssert(!s.contains(.one))
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }


    func testDistanceToSegment() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            let sqrt2 = (2 as Math.Scalar).squareRoot()

            let cases: [(lhs: S<Math>, rhs: S<Math>, distance: Math.Scalar)] = [
                // The same segments
                (lhs: S<Math>([ 0, 0 ], [ 1, 0 ]), rhs: S<Math>([ 0, 0 ], [ 1, 0 ]), distance: 0),
                // Intersecting arguments on the same line.
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([  1, 0 ], [ 3, 0, 0 ]), distance: 0),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([  2, 0 ], [ 4, 0, 0 ]), distance: 0),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([ -1, 0 ], [ 1, 0, 0 ]), distance: 0),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([ -2, 0 ], [ 0, 0, 0 ]), distance: 0),
                (lhs: S<Math>([ 1, 0 ], [ 2, 0 ]), rhs: S<Math>([  0, 0 ], [ 3, 0, 0 ]), distance: 0),
                (lhs: S<Math>([ 0, 0 ], [ 3, 0 ]), rhs: S<Math>([  1, 0 ], [ 2, 0, 0 ]), distance: 0),
                (lhs: S<Math>([ 0, 0 ], [ 3, 0 ]), rhs: S<Math>([  1, 0 ], [ 1, 0, 0 ]), distance: 0),
                // Intersecting arguments on different lines.
                (lhs: S<Math>([ 0, 0 ], [ 1, 0 ]), rhs: S<Math>([ 0, 0 ], [ 0, 1 ]), distance: 0),
                (lhs: S<Math>([ 0, 0 ], [ 1, 0 ]), rhs: S<Math>([ 1, 0 ], [ 1, 1 ]), distance: 0),
                (lhs: S<Math>([ 0, 1 ], [ 2, 1 ]), rhs: S<Math>([ 1, 0 ], [ 1, 2 ]), distance: 0),
                // Non-intersecting arguments on the same line.
                (lhs: S<Math>([ 0, 0 ], [ 1, 0 ]), rhs: S<Math>([ 2, 0 ], [ 3, 0 ]), distance: 1),
                // Collinear arguments.
                (lhs: S<Math>([ 2, 0 ], [ 3, 0 ]), rhs: S<Math>([ 2, 1 ], [ 5, 1 ]), distance: 1),
                (lhs: S<Math>([ 2, 0 ], [ 3, 0 ]), rhs: S<Math>([ 3, 1 ], [ 5, 1 ]), distance: 1),
                (lhs: S<Math>([ 2, 0 ], [ 3, 0 ]), rhs: S<Math>([ 4, 1 ], [ 5, 1 ]), distance: sqrt2),
                (lhs: S<Math>([ 2, 0 ], [ 3, 0 ]), rhs: S<Math>([ 0, 1 ], [ 2, 1 ]), distance: 1),
                (lhs: S<Math>([ 2, 0 ], [ 3, 0 ]), rhs: S<Math>([ 0, 1 ], [ 1, 1 ]), distance: sqrt2),
                // Non-collinear non-intersecting arguments.
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([ 1,  3 ], [ 5, -1 ]), distance: sqrt2),
                (lhs: S<Math>([ 2, 0 ], [ 0, 0 ]), rhs: S<Math>([ 1,  3 ], [ 5, -1 ]), distance: sqrt2),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([ 1, -3 ], [ 5,  1 ]), distance: sqrt2),
                (lhs: S<Math>([ 2, 0 ], [ 0, 0 ]), rhs: S<Math>([ 1, -3 ], [ 5,  1 ]), distance: sqrt2),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([ -1,  5 ], [  1,  3 ]), distance: 3),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([ -2,  6 ], [ -1,  5 ]), distance: Math.distance(Math.Vector2.zero, [ -1, 5 ])),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([  4,  0 ], [  5, -1 ]), distance: 2),
                (lhs: S<Math>([ 0, 0 ], [ 2, 0 ]), rhs: S<Math>([  2,  2 ], [  4,  0 ]), distance: sqrt2),
            ]

            cases.forEach { (lhs, rhs, expected) in
                let result = lhs.distance(to: rhs)
                XCTAssert(KvIs(result, equalTo: expected), "Value \(result) is not equal to expected distance \(expected) between segments «\(lhs)» and «\(rhs)»")
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }

}
