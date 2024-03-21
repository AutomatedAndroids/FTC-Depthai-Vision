plugins {
    id("com.android.library")
    id("org.jetbrains.kotlin.android")
}
var PUBLISH_GROUP_ID = "org.automatedandroids"
var PUBLISH_ARTIFACT_ID = "ftcdepthaivision"
var PUBLISH_VERSION = "0.0.1"

android {
    namespace = "com.example.ftc_depth_ai"
    compileSdk = 34

    defaultConfig {
        minSdk = 25

        testInstrumentationRunner = "androidx.test.runner.AndroidJUnitRunner"
        consumerProguardFiles("consumer-rules.pro")
        externalNativeBuild {
            cmake {
                cppFlags += ""
            }
        }
    }
    buildFeatures {
        android.buildFeatures.buildConfig=true
    }
    buildTypes {
        release {
            isMinifyEnabled = false
            proguardFiles(
                getDefaultProguardFile("proguard-android-optimize.txt"),
                "proguard-rules.pro"
            )
            ndk {
                //noinspection ChromeOsAbiSupport
                abiFilters += listOf("armeabi-v7a","arm64-v8a")
                ldLibs?.add("android")
            }
            buildConfigField("String","_VERSION_NAME","\""+ PUBLISH_VERSION + "\"")
        }
        debug {
            ndk {
                //noinspection ChromeOsAbiSupport
                abiFilters += listOf("armeabi-v7a","arm64-v8a")
            }
            buildConfigField("String","_VERSION_NAME","\"DEV BUILD\"")
        }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_1_8
        targetCompatibility = JavaVersion.VERSION_1_8
    }
    // Encapsulates your external native build configurations.
    externalNativeBuild {

        // Encapsulates your CMake build configurations.
        cmake {

            path = file("src/main/cpp/CMakeLists.txt")
            // Provides a relative path to your CMake build script.
            version = "3.22.1"
        }
    }
    ndkVersion = "21.3.6528147"
    kotlinOptions {
        jvmTarget = "1.8"
    }
}

dependencies {

    implementation("androidx.core:core-ktx:1.12.0")
    implementation("androidx.appcompat:appcompat:1.6.1")
    implementation("com.google.android.material:material:1.11.0")
    testImplementation("junit:junit:4.13.2")
    androidTestImplementation("androidx.test.ext:junit:1.1.5")
    androidTestImplementation("androidx.test.espresso:espresso-core:3.5.1")

    implementation("org.bytedeco:depthai:2.24.0-1.5.10")
    implementation("org.bytedeco:opencv:4.9.0-1.5.10")
    api("org.bytedeco:javacv:1.5.10")
    implementation("org.bytedeco:gradle-javacpp:1.5.10")


    // Internally recreated libs;
    // implementation("org.openftc:easyopencv:1.7.0")
    // implementation("org.openftc:apriltag:2.0.0")

    compileOnly("org.firstinspires.ftc:RobotCore:9.1.0")
}
