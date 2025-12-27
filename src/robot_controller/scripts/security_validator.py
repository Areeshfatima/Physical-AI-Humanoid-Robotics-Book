#!/usr/bin/env python3

"""
Security validation tool for ROS 2 packages.
This tool validates that ROS 2 communication implements authentication and encryption.
"""

import os
import sys
import subprocess
from pathlib import Path


def validate_ros2_security_environment():
    """
    Validates the ROS 2 security environment settings.
    """
    print("=== ROS 2 Security Environment Validation ===")
    
    # Check if security environment variables are set
    security_vars = {
        'ROS_SECURITY_ENABLE': os.environ.get('ROS_SECURITY_ENABLE'),
        'ROS_SECURITY_STRATEGY': os.environ.get('ROS_SECURITY_STRATEGY'),
        'ROS_SECURITY_ROOT_DIRECTORY': os.environ.get('ROS_SECURITY_ROOT_DIRECTORY')
    }
    
    print("Environment variables check:")
    all_set = True
    for var, value in security_vars.items():
        if value:
            print(f"  ✓ {var}={value}")
        else:
            print(f"  ✗ {var} is not set")
            all_set = False
    
    if not all_set:
        print("\nTo enable ROS 2 security, set these environment variables:")
        print("  export ROS_SECURITY_ENABLE=true")
        print("  export ROS_SECURITY_STRATEGY=Enforce")  # or Permissive
        print("  export ROS_SECURITY_ROOT_DIRECTORY=/path/to/security/files")
        print()
    
    return all_set


def check_security_files_exist():
    """
    Checks if required security files exist in the security directory.
    """
    security_dir = os.environ.get('ROS_SECURITY_ROOT_DIRECTORY')
    
    if not security_dir:
        print("ROS_SECURITY_ROOT_DIRECTORY not set, skipping security file check")
        return False
    
    security_dir = Path(security_dir)
    if not security_dir.exists():
        print(f"Security directory does not exist: {security_dir}")
        return False
    
    # Required security files for DDS security
    required_files = [
        'ca.cert.pem',           # Certificate Authority certificate
        'identity_ca.cert.pem',  # Identity CA certificate
        'permissions_ca.cert.pem',  # Permissions CA certificate
    ]
    
    print(f"\nChecking for security files in: {security_dir}")
    all_files_exist = True
    
    for file_name in required_files:
        file_path = security_dir / file_name
        if file_path.exists():
            print(f"  ✓ {file_name}")
        else:
            print(f"  ✗ {file_name} - MISSING")
            all_files_exist = False
    
    # Check for per-node key/cert files (pattern: NODE_NAME.cert.pem, NODE_NAME.key.pem)
    print("\nChecking for per-node security files:")
    node_files_found = False
    for file_path in security_dir.glob("*.cert.pem"):
        if "ca" not in file_path.name and "permissions" not in file_path.name:
            print(f"  ✓ Node certificate: {file_path.name}")
            node_files_found = True
    
    if not node_files_found:
        print("  ✗ No node certificates found")
        print("    (Each ROS 2 node should have its own certificate and key)")
    
    return all_files_exist


def validate_dds_security_config():
    """
    Validates DDS security configuration in ROS 2 packages.
    """
    print("\n=== DDS Security Configuration Validation ===")
    
    # Look for security configuration files in the workspace
    security_config_patterns = [
        "*.xml",  # Security permission files are typically XML
        "SECURITY*",  # Security-related files
    ]
    
    found_security_configs = False
    for root, dirs, files in os.walk("src"):
        for file in files:
            if file.endswith('.xml') and ('permission' in file.lower() or 'security' in file.lower()):
                print(f"  ✓ Found security config: {os.path.join(root, file)}")
                found_security_configs = True
                
    if not found_security_configs:
        print("  ✗ No DDS security configuration files found")
        print("    Consider creating security permission files for each node")
    
    return found_security_configs


def generate_security_config_skeleton():
    """
    Generates a skeleton security configuration file.
    """
    print("\n=== Generating Security Configuration Skeleton ===")
    
    # Example security permission file content
    permission_content = """<?xml version="1.0" encoding="UTF-8"?>
<dds xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
     xsi:noNamespaceSchemaLocation="http://www.omg.org/spec/DDS-SECURITY/20170901/omg_shared_ca_permissions.xsd">
    <permissions>
        <grant name="ROS2Participant">
            <subject_name>CN=ros2_ca,O=ROS,C=US</subject_name>
            <validity>
                <!-- Permissions are valid from 2023 to 2027 -->
                <not_before>2023-01-01T00:00:00</not_before>
                <not_after>2027-01-01T00:00:00</not_after>
            </validity>
            <allow_rule>
                <domains>
                    <id_range>
                        <min>0</min>
                        <max>230</max>
                    </id_range>
                </domains>
                <publish>
                    <topics>
                        <topic>/*</topic>
                    </topics>
                </publish>
                <subscribe>
                    <topics>
                        <topic>/*</topic>
                    </topics>
                </subscribe>
            </allow_rule>
            <default_deny>
            </default_deny>
        </grant>
    </permissions>
</dds>"""
    
    security_dir = os.environ.get('ROS_SECURITY_ROOT_DIRECTORY') or './security'
    os.makedirs(security_dir, exist_ok=True)
    
    perm_file = os.path.join(security_dir, 'permissions.xml')
    with open(perm_file, 'w') as f:
        f.write(permission_content)
    
    print(f"  Created security permissions template: {perm_file}")
    print("  Customize this file according to your security requirements")


def validate_package_security_implementation():
    """
    Validates specific security implementations in our packages.
    """
    print("\n=== Package Security Implementation Validation ===")
    
    # Check if nodes are prepared for security
    security_indicators = [
        ("publisher.py", ["qos", "QoS", "security"]),
        ("subscriber.py", ["qos", "QoS", "security"]),
        ("add_two_ints_server.py", ["qos", "QoS", "security"]),
        ("add_two_ints_client.py", ["qos", "QoS", "security"]),
        ("controller_manager.py", ["qos", "QoS", "security"]),
    ]
    
    packages_dir = Path("src")
    security_features_found = False
    
    for package_dir in packages_dir.iterdir():
        if package_dir.is_dir():
            for node_file, indicators in security_indicators:
                node_path = package_dir / "ros2_fundamentals" / node_file
                if node_path.exists():
                    content = node_path.read_text()
                    has_security = any(indicator in content.lower() for indicator in ['qos', 'durability', 'reliability'])
                    if has_security:
                        print(f"  ✓ {package_dir.name}/{node_file} - implements QoS profiles")
                        security_features_found = True
                    else:
                        print(f"  ⚠ {package_dir.name}/{node_file} - no QoS profiles found")
    
    if not security_features_found:
        print("  Consider implementing QoS profiles for security and reliability")
    
    return security_features_found


def main():
    """
    Main function to run all security validations.
    """
    print("ROS 2 Security Validation Tool")
    print("==============================")
    
    results = []
    results.append(validate_ros2_security_environment())
    results.append(check_security_files_exist())
    results.append(validate_dds_security_config())
    results.append(validate_package_security_implementation())
    
    # Generate skeleton if no security config exists
    if not any(results[2:]):  # If no DDS config found or package security
        generate_security_config_skeleton()
    
    print("\n=== Validation Summary ===")
    checks = [
        "Environment variables set",
        "Security files exist", 
        "DDS config present",
        "Package security implemented"
    ]
    
    for i, (check, result) in enumerate(zip(checks, results)):
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"  {status}: {check}")
    
    overall_pass = all(results)
    print(f"\nOverall: {'✓ PASS' if overall_pass else '✗ FAIL'}")
    
    if not overall_pass:
        print("\nSecurity validation failed. Please address the issues above.")
        print("For more information: https://docs.ros.org/en/rolling/Concepts/Security.html")
        return 1
    else:
        print("\nAll security validations passed!")
        return 0


if __name__ == "__main__":
    sys.exit(main())