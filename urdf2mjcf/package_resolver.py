"""Cross-platform ROS package path resolver.

This module provides utilities to resolve ROS package paths across different
ROS versions (ROS1/ROS2) and operating systems (Linux/Windows/macOS).
"""

import os
import platform
import logging
from pathlib import Path
from typing import Optional, Union, List

logger = logging.getLogger(__name__)


class PackageResolver:
    """Resolves ROS package paths using multiple fallback strategies."""
    
    def __init__(self):
        self._ros1_rospack = None
        self._ros2_available = None
        self._ros1_available = None
    
    def _init_ros1(self) -> bool:
        """Initialize ROS1 rospkg if available."""
        if self._ros1_available is not None:
            return self._ros1_available
            
        try:
            import rospkg
            self._ros1_rospack = rospkg.RosPack()
            self._ros1_available = True
            logger.debug("ROS1 rospkg initialized successfully")
            return True
        except ImportError:
            logger.debug("ROS1 rospkg not available")
            self._ros1_available = False
            return False
    
    def _init_ros2(self) -> bool:
        """Initialize ROS2 ament_index if available."""
        if self._ros2_available is not None:
            return self._ros2_available
            
        try:
            from ament_index_python.packages import get_package_share_directory
            self._ros2_available = True
            logger.debug("ROS2 ament_index_python initialized successfully")
            return True
        except ImportError:
            logger.debug("ROS2 ament_index_python not available")
            self._ros2_available = False
            return False
    
    def _find_workspace_root(self, start_path: Path) -> Optional[Path]:
        """
        Find the ROS workspace root by traversing up from the given path.
        
        Args:
            start_path: Path to start searching from (e.g., URDF file location)
            
        Returns:
            Path to workspace root, or None if not found
        """
        current = start_path.resolve()
        
        # Traverse up the directory tree
        while current.parent != current:  # Not at filesystem root
            # Check if this looks like a ROS workspace
            if any((current / d).exists() for d in ["src", "build", "devel", "install", "log"]):
                logger.debug(f"Found potential workspace root: {current}")
                return current
            
            # Check if we're in a src directory of a workspace
            if current.name == "src" and any((current.parent / d).exists() for d in ["build", "devel", "install"]):
                logger.debug(f"Found workspace root from src: {current.parent}")
                return current.parent
                
            current = current.parent
        
        return None
    
    def _recursive_find_package(self, search_dir: Path, package_name: str, max_depth: int = 5) -> Optional[Path]:
        """
        Recursively search for a package in the given directory.
        
        Args:
            search_dir: Directory to search in
            package_name: Name of the package to find
            max_depth: Maximum recursion depth
            
        Returns:
            Path to the package directory, or None if not found
        """
        if max_depth <= 0 or not search_dir.exists() or not search_dir.is_dir():
            return None
        
        # Check if current directory is the package we're looking for
        if search_dir.name == package_name and (search_dir / "package.xml").exists():
            logger.debug(f"Found package '{package_name}' at: {search_dir}")
            return search_dir
        
        # Recursively search subdirectories
        try:
            for item in search_dir.iterdir():
                if item.is_dir() and not item.name.startswith('.'):
                    # Skip common non-package directories
                    if item.name in ["build", "devel", "install", "log", "__pycache__", ".git"]:
                        continue
                    
                    result = self._recursive_find_package(item, package_name, max_depth - 1)
                    if result:
                        return result
        except (PermissionError, OSError):
            # Skip directories we can't read
            pass
        
        return None
    
    def _find_package_by_path_pattern(self, package_name: str, search_paths: Optional[List[Path]] = None) -> Optional[Path]:
        """Find package by searching common ROS workspace patterns."""
        if search_paths is None:
            search_paths = self._get_default_search_paths()
        
        for search_path in search_paths:
            if not search_path.exists():
                continue
                
            # First try direct patterns (fast)
            candidate_paths = [
                search_path / "src" / package_name,
                search_path / package_name,
                search_path / "share" / package_name,  # ROS2 install space
            ]
            
            for candidate in candidate_paths:
                if candidate.exists() and candidate.is_dir():
                    # Verify it's a ROS package by checking for package.xml
                    if (candidate / "package.xml").exists():
                        logger.debug(f"Found package '{package_name}' at: {candidate}")
                        return candidate
            
            # If direct patterns fail, try recursive search in src directory
            src_dir = search_path / "src"
            if src_dir.exists():
                result = self._recursive_find_package(src_dir, package_name)
                if result:
                    return result
            
            # Also try recursive search in the search_path itself (for non-standard layouts)
            result = self._recursive_find_package(search_path, package_name, max_depth=3)
            if result:
                return result
        
        return None
    
    def _get_default_search_paths(self) -> List[Path]:
        """Get default search paths based on operating system and environment."""
        search_paths = []
        
        # ROS environment variables
        ros_workspace_paths = [
            os.environ.get('ROS_WORKSPACE'),
            os.environ.get('COLCON_WS'),
            os.environ.get('CATKIN_WS'),
        ]
        
        for ws_path in ros_workspace_paths:
            if ws_path:
                search_paths.append(Path(ws_path))
        
        # Common ROS installation paths
        system = platform.system().lower()
        
        if system == "linux":
            # ROS1 and ROS2 standard installation paths
            search_paths.extend([
                Path("/opt/ros/noetic"),  # ROS1 Noetic
                Path("/opt/ros/melodic"),  # ROS1 Melodic
                Path("/opt/ros/humble"),   # ROS2 Humble
                Path("/opt/ros/iron"),     # ROS2 Iron
                Path("/opt/ros/jazzy"),    # ROS2 Jazzy
                Path("/opt/ros/rolling"),  # ROS2 Rolling
            ])
        elif system == "windows":
            # Windows ROS installation paths
            search_paths.extend([
                Path("C:/opt/ros/noetic"),
                Path("C:/opt/ros/humble"),
                Path("C:/dev/ros2"),
            ])
        elif system == "darwin":  # macOS
            # macOS ROS installation paths (often via Homebrew or custom)
            search_paths.extend([
                Path("/usr/local/opt/ros"),
                Path("/opt/homebrew/opt/ros"),
                Path(os.path.expanduser("~/ros")),
            ])
        
        # Current working directory and parent directories (for development)
        current_path = Path.cwd()
        search_paths.extend([
            current_path,
            current_path.parent,
            current_path.parent.parent,
        ])
        
        # Remove duplicates and non-existent paths
        unique_paths = []
        for path in search_paths:
            if path and path not in unique_paths:
                unique_paths.append(path)
        
        logger.debug(f"Default search paths: {unique_paths}")
        return unique_paths
    
    def resolve_package_path(self, package_name: str, search_paths: Optional[List[Union[str, Path]]] = None) -> Optional[Path]:
        """
        Resolve the path to a ROS package using multiple strategies.
        
        Args:
            package_name: Name of the ROS package
            search_paths: Optional list of additional paths to search
            
        Returns:
            Path to the package directory, or None if not found
        """
        logger.debug(f"Resolving package path for: {package_name}")
        
        # Strategy 1: Try ROS1 rospkg
        if self._init_ros1():
            try:
                path = self._ros1_rospack.get_path(package_name)
                logger.debug(f"Found package '{package_name}' via ROS1 rospkg: {path}")
                return Path(path)
            except Exception as e:
                logger.debug(f"ROS1 rospkg failed for package '{package_name}': {e}")
        
        # Strategy 2: Try ROS2 ament_index
        if self._init_ros2():
            try:
                from ament_index_python.packages import get_package_share_directory
                path = get_package_share_directory(package_name)
                logger.debug(f"Found package '{package_name}' via ROS2 ament_index: {path}")
                return Path(path)
            except Exception as e:
                logger.debug(f"ROS2 ament_index failed for package '{package_name}': {e}")
        
        # Strategy 3: Search by path patterns
        converted_search_paths = None
        if search_paths:
            converted_search_paths = [Path(p) for p in search_paths]
        
        path = self._find_package_by_path_pattern(package_name, converted_search_paths)
        if path:
            return path
        
        logger.warning(f"Could not resolve package path for: {package_name} in {converted_search_paths}")
        return None
    
    def resolve_package_resource(self, package_url: str, search_paths: Optional[List[Union[str, Path]]] = None) -> Optional[Path]:
        """
        Resolve a package:// URL to an absolute file path.
        
        Args:
            package_url: URL in format "package://package_name/path/to/resource"
            search_paths: Optional list of additional paths to search
            
        Returns:
            Absolute path to the resource, or None if not found
        """
        if not package_url.startswith("package://"):
            raise ValueError(f"Invalid package URL format: {package_url}")
        
        # Parse the package URL
        url_path = package_url[len("package://"):]
        parts = url_path.split("/", 1)
        
        if len(parts) < 2:
            raise ValueError(f"Invalid package URL format: {package_url}")
        
        package_name = parts[0]
        resource_path = parts[1]
        
        # Resolve package path
        package_path = self.resolve_package_path(package_name, search_paths)
        if package_path is None:
            return None
        
        # Construct full resource path
        full_path = package_path / resource_path
        
        if full_path.exists():
            logger.debug(f"Resolved package resource '{package_url}' to: {full_path}")
            return full_path
        else:
            logger.warning(f"Package resource not found: {full_path}")
            return None


# Global instance for easy access
_default_resolver = PackageResolver()


def resolve_package_path(package_name: str, search_paths: Optional[List[Union[str, Path]]] = None) -> Optional[Path]:
    """
    Convenience function to resolve a ROS package path.
    
    Args:
        package_name: Name of the ROS package
        search_paths: Optional list of additional paths to search
        
    Returns:
        Path to the package directory, or None if not found
    """
    return _default_resolver.resolve_package_path(package_name, search_paths)


def resolve_package_resource(package_url: str, search_paths: Optional[List[Union[str, Path]]] = None) -> Optional[Path]:
    """
    Convenience function to resolve a package:// URL to an absolute file path.
    
    Args:
        package_url: URL in format "package://package_name/path/to/resource"
        search_paths: Optional list of additional paths to search
        
    Returns:
        Absolute path to the resource, or None if not found
    """
    return _default_resolver.resolve_package_resource(package_url, search_paths)


def find_workspace_from_path(start_path: Union[str, Path]) -> Optional[Path]:
    """
    Find the ROS workspace root by traversing up from the given path.
    
    Args:
        start_path: Path to start searching from (e.g., URDF file location)
        
    Returns:
        Path to workspace root, or None if not found
    """
    return _default_resolver._find_workspace_root(Path(start_path)) 