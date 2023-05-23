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
//  KvSphere3Tests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 23.05.2023.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvSphere3Tests : XCTestCase {

    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: .testInit2Coordinates

    func testInit2Coordinates() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            typealias Scalar = Math.Scalar
            typealias Position = Math.Vector3
            typealias Sphere = KvSphere3<Math>

            let sqrt3: Scalar = (3.0 as Scalar).squareRoot()

            let one = Position.one
            let unitX = Position.unitX


            func Assert(_ p1: Position, _ p2: Position, _ expected: Sphere) {
                KvAssertEqual(Sphere(p1, p2), expected, "p1 = \(p1), p2 = \(p2)")
            }


            // Simple case with large magnitudes
            Assert((1e3 as Scalar) * one, (3e3 as Scalar) * one, .init(at: (2e3 as Scalar) * one, radius: (1e3 as Scalar) * sqrt3))
            // Degenerate case
            Assert(unitX, unitX, .init(at: unitX, radius: 0.0 as Scalar))
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: .testInit3Coordinates

    func testInit3Coordinates() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            typealias Scalar = Math.Scalar
            typealias Position = Math.Vector3
            typealias Sphere = KvSphere3<Math>

            let sqrt2: Scalar = (2.0 as Scalar).squareRoot()

            let zero = Position.zero
            let unitX = Position.unitX
            let unitY = Position.unitY


            func Assert(_ p1: Position, _ p2: Position, _ p3: Position, _ expected: Sphere?) {
                KvAssertEqual(Sphere(p1, p2, p3), expected, "p1 = \(p1), p2 = \(p2), p3 = \(p3)")
            }


            // Simple case with large magnitudes
            Assert((2e3 as Scalar) * unitX, (2e3 as Scalar) * unitY, (2e3 as Scalar) * (unitX + unitY), .init(at: (1e3 as Scalar) * (unitX + unitY), radius: (1e3 as Scalar) * sqrt2))
            // Duplicated coordinates
            Assert(zero, unitX, unitX, .init(at: (0.5 as Scalar) * unitX, radius: 0.5 as Scalar))
            Assert(unitX, zero, unitX, .init(at: (0.5 as Scalar) * unitX, radius: 0.5 as Scalar))
            Assert(unitX, unitX, zero, .init(at: (0.5 as Scalar) * unitX, radius: 0.5 as Scalar))
            // Degenerate case
            Assert(unitX, unitX, unitX, .init(at: unitX, radius: 0.0 as Scalar))
            // Incorrect case
            Assert(zero, unitX, (2.0 as Scalar) * unitX, nil)
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: .testInit4Coordinates

    func testInit4Coordinates() {

        func Run<Math : KvMathScope>(_ math: Math.Type) {
            typealias Scalar = Math.Scalar
            typealias Position = Math.Vector3
            typealias Sphere = KvSphere3<Math>

            let halfSqrt2: Scalar = (0.5 as Scalar) * (2.0 as Scalar).squareRoot()
            let sqrt3: Scalar = (3.0 as Scalar).squareRoot()

            let zero = Position.zero
            let one = Position.one
            let unitX = Position.unitX
            let unitY = Position.unitY
            let halfXY = (0.5 as Scalar) * (unitX + unitY)


            func Assert(_ p1: Position, _ p2: Position, _ p3: Position, _ p4: Position, _ expected: Sphere?) {
                KvAssertEqual(Sphere(p1, p2, p3, p4), expected, "p1 = \(p1), p2 = \(p2), p3 = \(p3), p4 = \(p4)")
            }


            // Simple case with large magnitudes
            Assert((2e3 as Scalar) * unitX, (2e3 as Scalar) * unitY, (2e3 as Scalar) * (unitX + unitY), (2e3 as Scalar) * one, .init(at: (1e3 as Scalar) * one, radius: (1e3 as Scalar) * sqrt3))
            // 2 duplicated coordinates
            Assert(zero,  zero, unitX, .unitY, .init(at: halfXY, radius: halfSqrt2))
            Assert(zero, unitX,  zero, .unitY, .init(at: halfXY, radius: halfSqrt2))
            Assert(zero, unitX, .unitY,  zero, .init(at: halfXY, radius: halfSqrt2))
            Assert(zero, unitX, unitX, .unitY, .init(at: halfXY, radius: halfSqrt2))
            Assert(zero, unitX, .unitY, unitX, .init(at: halfXY, radius: halfSqrt2))
            Assert(zero, unitX, unitY, .unitY, .init(at: halfXY, radius: halfSqrt2))
            // 3 duplicated coordinates
            Assert(zero, zero, zero, (2.0 as Scalar) * .unitX, .init(at: unitX, radius: 1.0 as Scalar))
            Assert(zero, zero, (2.0 as Scalar) * .unitX, zero, .init(at: unitX, radius: 1.0 as Scalar))
            Assert(zero, (2.0 as Scalar) * .unitX, zero, zero, .init(at: unitX, radius: 1.0 as Scalar))
            Assert((2.0 as Scalar) * .unitX, zero, zero, zero, .init(at: unitX, radius: 1.0 as Scalar))
            // Degenerate case
            Assert(unitX, unitX, unitX, unitX, .init(at: unitX, radius: 0.0 as Scalar))
            // Incorrect cases
            Assert(zero, unitX, (2.0 as Scalar) * unitX, (3.0 as Scalar) * unitX, nil)
            Assert(unitY, unitX, (2.0 as Scalar) * unitX, (3.0 as Scalar) * unitX, nil)
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: .testInit2Coordinates

    func testConains() {

        func Run<Math : KvMathScope>(_ math: Math.Type) where Math.Scalar.RawSignificand : FixedWidthInteger {
            typealias Scalar = Math.Scalar
            typealias Position = Math.Vector3
            typealias Sphere = KvSphere3<Math>

            let sphere = Sphere(at: [ 10.01 as Scalar, 8.77 as Scalar, 19.54 as Scalar ], radius: (60.0 as Scalar).squareRoot())

            let cases: [(t: Scalar, edgeExpected: Bool, insideExpected: Bool)] = [
                ( 0.0   , false, true ),
                ( 0.1   , false, true ),
                ( 0.999 , false, true ),
                ( 1.0   , true , false),
                ( 1.0001, false, false),
                ( 2.0   , false, false),
                (10.0   , false, false),
            ]

            (0 ..< 32).forEach { _ in
                let v = Math.Vector3.unitRandom()

                cases.forEach { (t, edgeExpected, insideExpected) in
                    let coordinate = sphere.center + t * sphere.radius * v

                    let coordinateDescription = { "Coordinate \(coordinate) (t = \(t), distance = \(Math.distance(sphere.center, coordinate)), radius: \(sphere.radius)" }

                    do {
                        XCTAssert(sphere.contains(coordinate) == edgeExpected, "\(coordinateDescription()) \(edgeExpected ? "is" : "is not") on the sphere's edge")
                    }
                    do {
                        var insideResult: Bool = false
                        XCTAssert(sphere.contains(coordinate, inside: &insideResult) == edgeExpected, "\(coordinateDescription()) \(edgeExpected ? "is" : "is not") on the sphere's edge")
                        XCTAssert(insideResult == insideExpected, "\(coordinateDescription()) \(insideExpected ? "is" : "is not") inside the sphere")
                    }
                }
            }
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }

}
