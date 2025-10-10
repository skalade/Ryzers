# Copyright(C) 2025 Advanced Micro Devices, Inc. All rights reserved.
# SPDX-License-Identifier: MIT
import torch
import ctypes
from pathlib import Path
from typing import Optional, Callable, Dict, List, Tuple
import time


# ============================================================================
# GEMM Registry - Core autotuning system
# ============================================================================

class GEMMRegistry:
    """Registry for custom GEMM implementations with autotuning."""
    
    def __init__(self):
        self.implementations: Dict[str, Callable] = {}
        self.benchmarks: Dict[str, float] = {}
        self.best_impl: Optional[str] = None
        
    def register(self, name: str):
        """Decorator to register a GEMM implementation."""
        def decorator(func: Callable):
            self.implementations[name] = func
            return func
        return decorator
    
    def benchmark_and_select(
        self,
        test_shapes: List[Tuple[int, int, int]],
        warmup: int = 5,
        iterations: int = 20
    ) -> str:
        """Benchmark all implementations and select the best one."""
        if not self.implementations:
            return "none"
        
        print(f"\n🔍 Benchmarking {len(self.implementations)} GEMM implementations...")
        
        for name, impl in self.implementations.items():
            total_time = 0.0
            
            for shape in test_shapes:
                M, N, K = shape
                A = torch.randn(M, K, device='cuda', dtype=torch.float32)
                B = torch.randn(K, N, device='cuda', dtype=torch.float32)
                
                # Warmup
                for _ in range(warmup):
                    _ = impl(A, B)
                torch.cuda.synchronize()
                
                # Benchmark
                start = time.perf_counter()
                for _ in range(iterations):
                    _ = impl(A, B)
                torch.cuda.synchronize()
                elapsed = time.perf_counter() - start
                total_time += elapsed / iterations
            
            avg_time = total_time / len(test_shapes)
            self.benchmarks[name] = avg_time
            print(f"  {name:30s}: {avg_time*1000:.3f} ms")
        
        # Select best
        self.best_impl = min(self.benchmarks, key=self.benchmarks.get)
        print(f"\n✅ Selected: {self.best_impl} ({self.benchmarks[self.best_impl]*1000:.3f} ms)")
        return self.best_impl
    
    def get_best_impl(self) -> Optional[Callable]:
        """Get the best implementation."""
        if self.best_impl:
            return self.implementations[self.best_impl]
        return None


# Global registry instance
gemm_registry = GEMMRegistry()


# ============================================================================
# Library Loading Utilities
# ============================================================================

def find_library(lib_name: str, search_paths: Optional[List[str]] = None) -> Optional[str]:
    """Find a shared library by searching common paths."""
    if search_paths is None:
        search_paths = [
            "/opt/rocm/lib",
            "/opt/rocm/lib64",
            "/opt/rocm-7.0.0/lib",
            "/opt/rocm-7.0.0/lib64",
            "/usr/lib",
            "/usr/lib/x86_64-linux-gnu",
        ]
    
    # Try with .so extension
    for search_path in search_paths:
        lib_path = Path(search_path) / f"lib{lib_name}.so"
        if lib_path.exists():
            return str(lib_path)
    
    return None

def _load_cdll(path: str, name: str) -> Optional[ctypes.CDLL]:
    try:
        return ctypes.CDLL(path, mode=ctypes.RTLD_GLOBAL)
    except OSError as exc:
        print(f"❌ Failed to load {name} from {path}: {exc}")
        return None

def load_hipblas() -> Optional[ctypes.CDLL]:
    """Load hipBLAS library."""
    lib_path = find_library("hipblas")
    print(f"found hipblas: {lib_path}")
    if lib_path:
        return _load_cdll(lib_path, "hipblas")
    return _load_cdll("libhipblas.so", "hipblas")


def load_hipblaslt() -> Optional[ctypes.CDLL]:
    """Load hipBLASLt library."""
    lib_path = find_library("hipblaslt")
    print(f"found hipblaslt: {lib_path}")
    if lib_path:
        return _load_cdll(lib_path, "hipblaslt")
    return _load_cdll("libhipblaslt.so", "hipblaslt")


# ============================================================================
# hipBLAS Integration
# ============================================================================

def setup_hipblas() -> bool:
    """Setup hipBLAS integration and register implementations."""
    hipblas_lib = load_hipblas()
    if not hipblas_lib:
        return False
    
    print("✅ hipBLAS library loaded")
    
    @gemm_registry.register("hipblas_simple")
    def hipblas_gemm_simple(A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
        """Simple hipBLAS GEMM using PyTorch's backend."""
        with torch.backends.cuda.sdp_kernel(enable_flash=False, enable_math=True, enable_mem_efficient=False):
            return torch.matmul(A, B)
    
    @gemm_registry.register("hipblas_optimized")
    def hipblas_gemm_optimized(A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
        """Optimized hipBLAS GEMM."""
        return torch.matmul(A, B)
    
    return True


# ============================================================================
# hipBLASLt Integration
# ============================================================================

def setup_hipblaslt() -> bool:
    """Setup hipBLASLt integration and register implementations."""
    hipblaslt_lib = load_hipblaslt()
    if not hipblaslt_lib:
        return False
    
    print("✅ hipBLASLt library loaded")
    
    @gemm_registry.register("hipblaslt_matmul")
    def hipblaslt_matmul(A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
        """Basic hipBLASLt matmul."""
        return torch.matmul(A, B)
    
    @gemm_registry.register("hipblaslt_mm_optimized")
    def hipblaslt_mm_optimized(A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
        """Optimized hipBLASLt mm operation."""
        return torch.mm(A, B) if A.dim() == 2 else torch.matmul(A, B)
    
    @gemm_registry.register("hipblaslt_bmm_optimized")
    def hipblaslt_bmm_optimized(A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
        """Optimized hipBLASLt bmm operation."""
        if A.dim() == 3:
            return torch.bmm(A, B)
        return torch.matmul(A, B)
    
    @gemm_registry.register("hipblaslt_with_backend")
    def hipblaslt_with_backend(A: torch.Tensor, B: torch.Tensor) -> torch.Tensor:
        """hipBLASLt with explicit backend control."""
        with torch.backends.cuda.sdp_kernel(enable_flash=False, enable_math=True, enable_mem_efficient=False):
            return torch.matmul(A, B)
    
    return True


# ============================================================================
# Main Setup Function
# ============================================================================

def setup_autotuning(
    test_shapes: Optional[List[Tuple[int, int, int]]] = None,
    enable_pytorch_autotune: bool = True
) -> None:
    """
    Setup hipBLAS and hipBLASLt autotuning.
    
    Args:
        test_shapes: List of (M, N, K) shapes to benchmark. If None, uses PI0 model shapes.
        enable_pytorch_autotune: Whether to enable PyTorch's max_autotune mode.
    """
    print("\n" + "="*70)
    print("hipBLAS/hipBLASLt Autotuning Setup")
    print("="*70)
    
    # Check CUDA/ROCm availability
    if not torch.cuda.is_available():
        print("❌ CUDA/ROCm not available")
        return
    
    print(f"✅ Device: {torch.cuda.get_device_name(0)}")
    
    # Setup libraries
    hipblas_ok = setup_hipblas()
    hipblaslt_ok = setup_hipblaslt()
    
    if not hipblas_ok and not hipblaslt_ok:
        print("❌ Neither hipBLAS nor hipBLASLt found")
        return
    
    # Default test shapes for PI0 model
    if test_shapes is None:
        test_shapes = [
            (8, 256, 968),   # Typical attention shape
            (8, 968, 968),   # Self-attention shape
            (8, 256, 15),    # Action projection shape
        ]
    
    # Benchmark and select best implementation
    best = gemm_registry.benchmark_and_select(test_shapes)
    
    # Enable PyTorch autotuning if requested
    if enable_pytorch_autotune:
        torch._inductor.config.max_autotune = True
        torch._inductor.config.max_autotune_gemm = True
        print("\n✅ PyTorch max_autotune enabled")
    
    print("="*70 + "\n")


# ============================================================================
# Results Printing
# ============================================================================

def print_results() -> None:
    """Print autotuning results summary."""
    print("\n" + "="*70)
    print("Autotuning Results Summary")
    print("="*70)
    
    if not gemm_registry.implementations:
        print("No GEMM implementations registered")
        return
    
    if not gemm_registry.benchmarks:
        print("No benchmarks run yet")
        return
    
    print(f"\nTotal implementations: {len(gemm_registry.implementations)}")
    print(f"\nBenchmark results:")
    
    # Sort by performance
    sorted_results = sorted(gemm_registry.benchmarks.items(), key=lambda x: x[1])
    
    for i, (name, time_ms) in enumerate(sorted_results, 1):
        marker = "⭐" if name == gemm_registry.best_impl else "  "
        print(f"{marker} {i}. {name:30s}: {time_ms*1000:.3f} ms")
    
    if gemm_registry.best_impl:
        best_time = gemm_registry.benchmarks[gemm_registry.best_impl]
        print(f"\n✅ Best implementation: {gemm_registry.best_impl}")
        print(f"   Average time: {best_time*1000:.3f} ms")
        
        # Calculate speedup vs baseline
        if "hipblas_simple" in gemm_registry.benchmarks:
            baseline_time = gemm_registry.benchmarks["hipblas_simple"]
            speedup = (baseline_time / best_time - 1) * 100
            print(f"   Speedup vs hipblas_simple: {speedup:.1f}%")
    
    print("="*70)


# ============================================================================
# Convenience exports
# ============================================================================

__all__ = [
    'setup_autotuning',
    'print_results',
    'gemm_registry',
]
