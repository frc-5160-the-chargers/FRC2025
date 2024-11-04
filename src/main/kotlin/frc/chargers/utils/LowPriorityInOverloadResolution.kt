@file:Suppress("PackageDirectoryMismatch")
package kotlin.internal

/**
 * An annotation that marks a function with lower priority in overload resolution.
 * Part of the kotlin internal package.
 *
 * If the compiler has trouble selecting a specific overload of a function,
 * mark the lesser-priority one with this annotation.
 */
@Target(AnnotationTarget.FUNCTION, AnnotationTarget.PROPERTY, AnnotationTarget.CONSTRUCTOR)
@Retention(AnnotationRetention.BINARY)
internal annotation class LowPriorityInOverloadResolution
