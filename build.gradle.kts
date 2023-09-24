plugins {
    kotlin("jvm") version "1.9.0"
    application
}

group = "com.josephcatrambone"
version = "0.1"
val boofVersion = "0.39"

repositories {
    mavenCentral()
}

dependencies {
    testImplementation(kotlin("test"))

    implementation("org.jetbrains.kotlinx:kotlinx-coroutines-core:1.7.3")

    arrayOf("core","swing","kotlin","WebcamCapture").forEach() {
        implementation("org.boofcv:boofcv-$it:$boofVersion")
    }
}

tasks.test {
    useJUnitPlatform()
}

kotlin {
    jvmToolchain(8)
}

application {
    mainClass.set("MainKt")
}