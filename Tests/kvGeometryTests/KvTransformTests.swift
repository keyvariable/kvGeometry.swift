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
//  KvTransformTests.swift
//  kvGeometry
//
//  Created by Svyatoslav Popov on 11.10.2022.
//

import XCTest

@testable import kvGeometry

import kvKit
import kvTestKit



class KvTransformTests : XCTestCase {

    typealias T2<Math : KvMathScope> = KvTransform2<Math>
    typealias AT2<Math : KvMathScope> = KvAffineTransform2<Math>

    typealias T3<Math : KvMathScope> = KvTransform3<Math>
    typealias AT3<Math : KvMathScope> = KvAffineTransform3<Math>



    // MARK: : XCTestCase

    override func setUpWithError() throws {
        // Put setup code here. This method is called before the invocation of each test method in the class.
    }


    override func tearDownWithError() throws {
        // Put teardown code here. This method is called after the invocation of each test method in the class.
    }



    // MARK: Scale Tests

    func testS() {

        func RunT2<Math : KvMathScope>(_ math: Math.Type, scales: Math.Vector2...) {
            scales.forEach { scale in
                KvAssertEqual(T2<Math>(scale: scale),
                              .init(s3(Math.self, scale)),
                              "(scale: \(scale))")
                KvAssertEqual(AT2<Math>(scale: scale),
                              .init(s2(Math.self, scale)),
                              "(scale: \(scale))")
            }
        }

        func RunT3<Math : KvMathScope>(_ math: Math.Type, scales: Math.Vector3...) {
            scales.forEach { scale in
                KvAssertEqual(T3<Math>(scale: scale),
                              .init(s4(Math.self, scale)),
                              "(scale: \(scale))")
                KvAssertEqual(AT3<Math>(scale: scale),
                              .init(s3(Math.self, scale)),
                              "(scale: \(scale))")
            }
        }

        // TODO: Complete tests
        func Run<Math : KvMathScope>(_ math: Math.Type) {
            RunT2(
                math,
                scales: .one, [ 2, -2 ]
            )

            RunT3(
                math,
                scales: .one, [ 2, -2, 2 ]
            )
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Rotation Tests

    func testR() {

        func RunT2<Math : KvMathScope>(_ math: Math.Type, angles: Math.Scalar...) {
            angles.forEach { angle in
                KvAssertEqual(T2<Math>(angle: angle),
                              .init(r3(Math.self, angle)),
                              "(angle: \(angle))")
                KvAssertEqual(AT2<Math>(angle: angle),
                              .init(r2(Math.self, angle)),
                              "(angle: \(angle))")
            }
        }

        func RunT3<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector3..., quaternions: Math.Quaternion..., scales: Math.Vector3...) {
            quaternions.forEach { quaternion in
                KvAssertEqual(T3<Math>(quaternion: quaternion),
                              .init(r4(Math.self, quaternion)),
                              "(quaternion: (angle: \(quaternion.angle / .pi) * .pi, axis: \(quaternion.axis)))")
                KvAssertEqual(AT3<Math>(quaternion: quaternion),
                              .init(r3(Math.self, quaternion)),
                              "(quaternion: (angle: \(quaternion.angle / .pi) * .pi, axis: \(quaternion.axis)))")
            }
        }

        // TODO: Complete tests
        func Run<Math : KvMathScope>(_ math: Math.Type) {
            typealias Scalar = Math.Scalar

            RunT2(
                math,
                angles: 0.0 as Scalar, 0.1 * Scalar.pi, 0.25 * Scalar.pi, 0.5 * Scalar.pi, Scalar.pi
            )

            RunT3(
                math,
                quaternions: .zeroAngle, .init(angle: 0.1 * Scalar.pi, axis: .unitX), .init(angle: 0.25 * Scalar.pi, axis: .unitY), .init(angle: 0.5 * Scalar.pi, axis: .unitZ)
            )
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Translation Tests

    func testT() {

        func RunT2<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector2...) {
            translations.forEach { translation in
                KvAssertEqual(T2<Math>(translation: translation),
                              .init(t(Math.self, translation)),
                              "(translation: \(translation))")
            }
        }

        func RunT3<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector3...) {
            translations.forEach { translation in
                KvAssertEqual(T3<Math>(translation: translation),
                              .init(t(Math.self, translation)),
                              "(translation: \(translation))")
            }
        }

        // TODO: Complete tests
        func Run<Math : KvMathScope>(_ math: Math.Type) {
            RunT2(
                math,
                translations: .zero, [ 1, 3 ]
            )

            RunT3(
                math,
                translations: .zero, [ 1, 3, 5 ]
            )
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Scale+Rotation Tests

    func testRS() {

        func RunT2<Math : KvMathScope>(_ math: Math.Type, angles: Math.Scalar..., scales: Math.Vector2...) {
            angles.forEach { angle in
                scales.forEach { scale in
                    KvAssertEqual(T2<Math>(angle: angle, scale: scale),
                                  .init(r3(Math.self, angle) * s3(Math.self, scale)),
                                  "(scale: \(scale), angle: \(angle))")
                    KvAssertEqual(AT2<Math>(angle: angle, scale: scale),
                                  .init(r2(Math.self, angle) * s2(Math.self, scale)),
                                  "(scale: \(scale), angle: \(angle))")
                }
            }
        }

        func RunT3<Math : KvMathScope>(_ math: Math.Type, quaternions: Math.Quaternion..., scales: Math.Vector3...) {
            quaternions.forEach { quaternion in
                scales.forEach { scale in
                    KvAssertEqual(T3<Math>(quaternion: quaternion, scale: scale),
                                  .init(r4(Math.self, quaternion) * s4(Math.self, scale)),
                                  "(scale: \(scale), quaternion: (angle: \(quaternion.angle / .pi) * .pi, axis: \(quaternion.axis)))")
                    KvAssertEqual(AT3<Math>(quaternion: quaternion, scale: scale),
                                  .init(r3(Math.self, quaternion) * s3(Math.self, scale)),
                                  "(scale: \(scale), quaternion: (angle: \(quaternion.angle / .pi) * .pi, axis: \(quaternion.axis)))")
                }
            }
        }

        // TODO: Complete tests
        func Run<Math : KvMathScope>(_ math: Math.Type) {
            RunT2(
                math,
                angles: 0, 0.1 * .pi, 0.25 * .pi, 0.5 * .pi, .pi,
                scales: .one, [ 2, -2 ]
            )

            RunT3(
                math,
                quaternions: .zeroAngle, .init(angle: 0.1 * .pi, axis: .unitX), .init(angle: 0.25 * .pi, axis: .unitY), .init(angle: 0.5 * .pi, axis: .unitZ),
                scales: .one, [ 2, -2, 2 ]
            )
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Scale+Translation Tests

    func testTS() {

        func RunT2<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector2..., scales: Math.Vector2...) {
            translations.forEach { translation in
                scales.forEach { scale in
                    KvAssertEqual(T2<Math>(translation: translation, scale: scale),
                                  .init(t(Math.self, translation) * s3(Math.self, scale)),
                                  "(scale: \(scale), translation: \(translation))")
                }
            }
        }

        func RunT3<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector3..., scales: Math.Vector3...) {
            translations.forEach { translation in
                scales.forEach { scale in
                    KvAssertEqual(T3<Math>(translation: translation, scale: scale),
                                  .init(t(Math.self, translation) * s4(Math.self, scale)),
                                  "(scale: \(scale), translation: \(translation))")
                }
            }
        }

        // TODO: Complete tests
        func Run<Math : KvMathScope>(_ math: Math.Type) {
            RunT2(
                math,
                translations: .zero, [ 1, 3 ],
                scales: .one, [ 2, -2 ]
            )

            RunT3(
                math,
                translations: .zero, [ 1, 3, 5 ],
                scales: .one, [ 2, -2, 2 ]
            )
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Rotation+Translation Tests

    func testTR() {

        func RunT2<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector2..., angles: Math.Scalar...) {
            translations.forEach { translation in
                angles.forEach { angle in
                    KvAssertEqual(T2<Math>(translation: translation, angle: angle),
                                  .init(t(Math.self, translation) * r3(Math.self, angle)),
                                  "(angle: \(angle), translation: \(translation))")
                }
            }
        }

        func RunT3<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector3..., quaternions: Math.Quaternion...) {
            translations.forEach { translation in
                quaternions.forEach { quaternion in
                    KvAssertEqual(T3<Math>(translation: translation, quaternion: quaternion),
                                  .init(t(Math.self, translation) * r4(Math.self, quaternion)),
                                  "(quaternion: (angle: \(quaternion.angle / .pi) * .pi, axis: \(quaternion.axis)), translation: \(translation))")
                }
            }
        }

        // TODO: Complete tests
        func Run<Math : KvMathScope>(_ math: Math.Type) {
            RunT2(math,
                  translations: .zero, [ 1, 3 ],
                  angles: 0, 0.1 * .pi, 0.25 * .pi, 0.5 * .pi, .pi)

            RunT3(math,
                  translations: .zero, [ 1, 3, 5 ],
                  quaternions: .zeroAngle, .init(angle: 0.1 * .pi, axis: .unitX), .init(angle: 0.25 * .pi, axis: .unitY), .init(angle: 0.5 * .pi, axis: .unitZ))
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Scale+Rotation+Translation Tests

    func testTRS() {

        func RunT2<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector2..., angles: Math.Scalar..., scales: Math.Vector2...) {
            translations.forEach { translation in
                angles.forEach { angle in
                    scales.forEach { scale in
                        KvAssertEqual(T2<Math>(translation: translation, angle: angle, scale: scale),
                                      .init(t(Math.self, translation) * r3(Math.self, angle) * s3(Math.self, scale)),
                                      "(scale: \(scale), angle: \(angle), translation: \(translation))")
                    }
                }
            }
        }

        func RunT3<Math : KvMathScope>(_ math: Math.Type, translations: Math.Vector3..., quaternions: Math.Quaternion..., scales: Math.Vector3...) {
            translations.forEach { translation in
                quaternions.forEach { quaternion in
                    scales.forEach { scale in
                        KvAssertEqual(T3<Math>(translation: translation, quaternion: quaternion, scale: scale),
                                      .init(t(Math.self, translation) * r4(Math.self, quaternion) * s4(Math.self, scale)),
                                      "(scale: \(scale), quaternion: (angle: \(quaternion.angle / .pi) * .pi, axis: \(quaternion.axis)), translation: \(translation))")
                    }
                }
            }
        }

        // TODO: Complete tests
        func Run<Math : KvMathScope>(_ math: Math.Type) {
            RunT2(
                math,
                translations: .zero, [ 1, 3 ],
                angles: 0, 0.1 * .pi, 0.25 * .pi, 0.5 * .pi, .pi,
                scales: .one, [ 2, -2 ]
            )

            RunT3(
                math,
                translations: .zero, [ 1, 3, 5 ],
                quaternions: .zeroAngle, .init(angle: 0.1 * .pi, axis: .unitX), .init(angle: 0.25 * .pi, axis: .unitY), .init(angle: 0.5 * .pi, axis: .unitZ),
                scales: .one, [ 2, -2, 2 ]
            )
        }

        Run(KvMathFloatScope.self)
        Run(KvMathDoubleScope.self)
    }



    // MARK: Auxliaries

    private func s2<Math : KvMathScope>(_ math: Math.Type, _ scale: Math.Vector2) -> Math.Matrix2x2 {
        AT2<Math>.makeMatrix(scale: scale)
    }

    private func s3<Math : KvMathScope>(_ math: Math.Type, _ scale: Math.Vector2) -> Math.Matrix3x3 {
        T2<Math>.makeMatrix(scale: scale)
    }

    private func s3<Math : KvMathScope>(_ math: Math.Type, _ scale: Math.Vector3) -> Math.Matrix3x3 {
        AT3<Math>.makeMatrix(scale: scale)
    }

    private func s4<Math : KvMathScope>(_ math: Math.Type, _ scale: Math.Vector3) -> Math.Matrix4x4 {
        T3<Math>.makeMatrix(scale: scale)
    }


    private func r2<Math : KvMathScope>(_ math: Math.Type, _ angle: Math.Scalar) -> Math.Matrix2x2 {
        AT2<Math>.makeMatrix(angle: angle)
    }

    private func r3<Math : KvMathScope>(_ math: Math.Type, _ angle: Math.Scalar) -> Math.Matrix3x3 {
        T2<Math>.makeMatrix(angle: angle)
    }

    private func r3<Math : KvMathScope>(_ math: Math.Type, _ quaternion: Math.Quaternion) -> Math.Matrix3x3 {
        AT3<Math>.makeMatrix(quaternion: quaternion)
    }

    private func r4<Math : KvMathScope>(_ math: Math.Type, _ quaternion: Math.Quaternion) -> Math.Matrix4x4 {
        T3<Math>.makeMatrix(quaternion: quaternion)
    }


    private func t<Math : KvMathScope>(_ math: Math.Type, _ translation: Math.Vector2) -> T2<Math>.Matrix {
        T2<Math>.makeMatrix(translation: translation)
    }

    private func t<Math : KvMathScope>(_ math: Math.Type, _ translation: Math.Vector3) -> T3<Math>.Matrix {
        T3<Math>.makeMatrix(translation: translation)
    }

}
