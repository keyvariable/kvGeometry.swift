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
//  KvLine2Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 28.09.2022.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvLine2Tests : XCTestCase {

    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: Intersection with Line Test

    func testIntersectionWithLine() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            let c = -(0.5 as Math.Scalar).squareRoot()
            let line = makeLine(math, angle: (0.25 as Math.Scalar) * Math.Scalar.pi, c: c)

            func Assert(_ result: Math.Vector2?, _ expected: Math.Vector2?) {
                KvAssertEqual(result, expected, by: Math.isEqual(_:_:))
            }

            Assert(line.intersection(with: makeLine(math, angle: 0, c: 0)), [ -1, 0 ])
            Assert(line.intersection(with: makeLine(math, angle: .pi, c: 2)), [ 1, 2 ])
            Assert(line.intersection(with: makeLine(math, angle:  0.5 * .pi, c: 0)), [ 0, 1 ])
            Assert(line.intersection(with: makeLine(math, angle: -0.5 * .pi, c: 0)), [ 0, 1 ])
            Assert(line.intersection(with: makeLine(math, angle: 0.25 * .pi, c: 0)), nil)
            Assert(line.intersection(with: makeLine(math, angle: 1.25 * .pi, c: 0)), nil)
            Assert(line.intersection(with: makeLine(math, angle: 0.25 * .pi, c: c)), nil)
            Assert(line.intersection(with: makeLine(math, angle: 1.25 * .pi, c: c)), nil)
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: .contains Coordiante Tests

    func testContainsCoordinate() {

        func Run<Math : KvMathScope>(_ math: Math.Type)
        where Math.Scalar.RawSignificand : FixedWidthInteger
        {
            typealias Scalar = Math.Scalar

            (0..<50).forEach { _ in
                let line = KvLine2<Math>(in: Math.randomNonzero2(in: (-10.0 as Scalar) ... (10.0 as Scalar)),
                                         at: Math.random2(in: (-100.0 as Scalar) ... (100.0 as Scalar)))

                var t = -100.0 as Scalar
                while t <= 100.0 as Scalar {
                    defer { t += 25.0 as Scalar }

                    let c_in = line.closestToOrigin + t * line.front
                    XCTAssert(line.contains(c_in), "contains: line = (in: \(line.front), at: \(line.closestToOrigin)), c = \(c_in)")

                    let offset: Scalar = max(1.0 as Scalar, Math.abs(c_in).max()) * (1e-2 as Scalar)
                    let c_out = c_in + offset * line.normal
                    XCTAssert(!line.contains(c_out), "!contains: line = (in: \(line.front), at: \(line.closestToOrigin)), c = \(c_out)")
                }
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Auxiliaries

    private func makeLine<Math : KvMathScope>(_ math: Math.Type, angle: Math.Scalar, c: Math.Scalar) -> KvLine2<Math> {
        let (sine, cosine) = Math.sincos(angle + (0.5 as Math.Scalar) * Math.Scalar.pi)
        return .init(normal: .init(x: cosine, y: sine), c: c)
    }

}
