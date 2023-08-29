rmdir /s /q build
mkdir build
cd build
cmake ..
cmake --build .
@REM start PhysicsEngine.sln

@REM Run tests if built
if exist test\Debug\tests.exe (
    echo Running tests...
    test\Debug\tests.exe
) else (
    echo tests not built, exiting...
)

cd ..