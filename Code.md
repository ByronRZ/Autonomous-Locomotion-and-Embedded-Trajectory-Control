---
layout: default
title: Code
nav_order: 3
---

# Design of Autonomous Locomotion and Embedded Trajectory Control
```cpp
echo &#39;#include &lt;DifferentialNeun.h&gt;
#include &lt;iostream&gt;

int main() {
    // Inicializar neurona
    Neuron n(args);
    std::cout &lt;&lt; &quot;Neun C++ lib&quot;;
    return 0;
} &#39; &gt; test_neun.cpp

g++ -std=c++20 test_neun.cpp
./test_neun

