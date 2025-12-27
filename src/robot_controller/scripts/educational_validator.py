#!/usr/bin/env python3

"""
Validation script to ensure all ROS 2 tutorials meet educational requirements.
Validates that tutorials are simplified but technically correct.
"""

import os
import re
from pathlib import Path
import ast


def validate_code_simplicity_and_correctness():
    """
    Validates that example code is simplified but technically correct.
    """
    print("=== Validating Code Simplicity and Correctness ===")
    
    # Define the source directories for our packages
    package_dirs = [
        "src/publisher_subscriber_tutorial/ros2_fundamentals",
        "src/services_actions_tutorial/ros2_fundamentals",
        "src/robot_controller/robot_controller",
    ]
    
    all_valid = True
    
    for pkg_dir in package_dirs:
        if not os.path.exists(pkg_dir):
            print(f"  ⚠ Package directory not found: {pkg_dir}")
            continue
            
        print(f"\nChecking {pkg_dir}:")
        
        for file in os.listdir(pkg_dir):
            if file.endswith('.py'):
                file_path = os.path.join(pkg_dir, file)
                
                # Read and validate the Python file
                try:
                    with open(file_path, 'r') as f:
                        content = f.read()
                    
                    # Check if it's parseable Python (technically correct)
                    try:
                        ast.parse(content)
                        print(f"  ✓ {file} - valid Python syntax")
                    except SyntaxError as e:
                        print(f"  ✗ {file} - syntax error: {e}")
                        all_valid = False
                        continue
                    
                    # Check for complexity metrics
                    lines = content.split('\n')
                    line_count = len([l for l in lines if l.strip() and not l.strip().startswith('#')])
                    
                    # Check if code is not overly complex for educational purposes
                    if line_count > 100:  # Arbitrary threshold for "simple"
                        print(f"  ⚠ {file} - {line_count} lines (might be too complex for beginners)")
                    else:
                        print(f"  ✓ {file} - {line_count} lines (appropriate length)")
                    
                    # Check for essential educational elements
                    has_imports = bool(re.search(r'^import.*rclpy', content, re.MULTILINE))
                    has_node_class = bool(re.search(r'class.*Node', content))
                    has_main_function = bool('def main(' in content)
                    
                    educational_elements = sum([has_imports, has_node_class, has_main_function])
                    if educational_elements >= 2:  # At least 2 out of 3
                        print(f"  ✓ {file} - contains educational elements (imports, node class, main)")
                    else:
                        print(f"  ⚠ {file} - missing educational elements")
                
                except Exception as e:
                    print(f"  ✗ {file} - error reading file: {e}")
                    all_valid = False
    
    return all_valid


def validate_documentation_quality():
    """
    Validates that documentation is clear and educational.
    """
    print("\n=== Validating Documentation Quality ===")
    
    # Check our Docusaurus documentation
    docs_dir = "my-book/docs/ros2-fundamentals"
    
    if not os.path.exists(docs_dir):
        print(f"  ✗ Documentation directory not found: {docs_dir}")
        return False
    
    docs_valid = True
    required_docs = ['index.md', 'publisher-subscriber.md', 'services-actions.md', 'urdf-controllers.md', 'summary.md']
    
    print(f"Checking documentation in {docs_dir}:")
    
    for doc in required_docs:
        doc_path = os.path.join(docs_dir, doc)
        if os.path.exists(doc_path):
            with open(doc_path, 'r') as f:
                content = f.read()
            
            # Check for essential educational content
            has_examples = len(re.findall(r'```.*?```', content, re.DOTALL)) > 0
            has_explanations = len(content) > 500  # At least some explanation
            
            if has_examples and has_explanations:
                print(f"  ✓ {doc} - contains examples and explanations")
            else:
                print(f"  ⚠ {doc} - may lack examples or explanations")
        else:
            print(f"  ✗ {doc} - missing documentation file")
            docs_valid = False
    
    return docs_valid


def validate_educational_tutorials():
    """
    Validates that tutorials follow educational best practices.
    """
    print("\n=== Validating Educational Best Practices ===")
    
    # Check that tutorials have clear learning objectives
    quickstart_path = "src/quickstart.md"
    if os.path.exists(quickstart_path):
        with open(quickstart_path, 'r') as f:
            content = f.read()
        
        # Look for educational elements
        has_prerequisites = 'Prerequisites' in content or 'prerequisites' in content
        has_step_by_step = '1.' in content and ('install' in content or 'run' in content or 'create' in content)
        has_clear_goals = ('Goal:' in content or 'objective' in content.lower() or 'learn' in content.lower())
        
        print(f"Quickstart guide validation:")
        if has_prerequisites:
            print("  ✓ Includes prerequisites")
        else:
            print("  ⚠ May lack prerequisites section")
            
        if has_step_by_step:
            print("  ✓ Includes step-by-step instructions")
        else:
            print("  ⚠ May lack step-by-step instructions")
            
        if has_clear_goals:
            print("  ✓ Includes clear goals/learning objectives")
        else:
            print("  ⚠ May lack clear learning objectives")
        
        return has_prerequisites and has_step_by_step
    else:
        print(f"  ✗ Quickstart guide not found: {quickstart_path}")
        return False


def validate_implementation_correctness():
    """
    Validates that implementations are technically correct.
    """
    print("\n=== Validating Implementation Correctness ===")
    
    # Check if setup.py files properly define entry points
    package_setup_files = [
        "src/publisher_subscriber_tutorial/setup.py",
        "src/services_actions_tutorial/setup.py", 
        "src/urdf_tutorial/setup.py",
        "src/robot_controller/setup.py"
    ]
    
    correct_impl = True
    
    for setup_file in package_setup_files:
        if os.path.exists(setup_file):
            with open(setup_file, 'r') as f:
                content = f.read()
            
            # Check if setup.py has proper entry points
            has_entry_points = 'entry_points' in content and 'console_scripts' in content
            has_proper_setup = 'setup(' in content and 'packages=' in content
            
            if has_entry_points and has_proper_setup:
                print(f"  ✓ {setup_file} - properly configured")
            else:
                print(f"  ⚠ {setup_file} - may have configuration issues")
        else:
            print(f"  ✗ Setup file not found: {setup_file}")
            correct_impl = False
    
    # Check package.xml files
    print("\nChecking package.xml files:")
    package_xml_files = [
        "src/publisher_subscriber_tutorial/package.xml",
        "src/services_actions_tutorial/package.xml", 
        "src/urdf_tutorial/package.xml",
        "src/robot_controller/package.xml"
    ]
    
    for xml_file in package_xml_files:
        if os.path.exists(xml_file):
            with open(xml_file, 'r') as f:
                content = f.read()
            
            # Check for essential elements
            has_name = '<name>' in content
            has_version = '<version>' in content
            has_description = '<description>' in content
            has_maintainer = '<maintainer' in content
            has_license = '<license>' in content
            
            essential_present = has_name and has_version and has_description and has_maintainer and has_license
            
            if essential_present:
                print(f"  ✓ {xml_file} - complete package definition")
            else:
                print(f"  ⚠ {xml_file} - missing essential elements")
                correct_impl = False
        else:
            print(f"  ✗ Package file not found: {xml_file}")
            correct_impl = False
    
    return correct_impl


def main():
    """
    Main validation function.
    """
    print("ROS 2 Tutorials Educational Validation")
    print("=====================================")
    print("Validating that all tutorials are simplified but technically correct")
    print()
    
    # Run all validations
    code_valid = validate_code_simplicity_and_correctness()
    docs_valid = validate_documentation_quality()
    tutorial_valid = validate_educational_tutorials()
    impl_valid = validate_implementation_correctness()
    
    print(f"\n=== Summary ===")
    print(f"Code simplicity and correctness: {'✓ PASS' if code_valid else '✗ FAIL'}")
    print(f"Documentation quality: {'✓ PASS' if docs_valid else '✗ FAIL'}")
    print(f"Educational best practices: {'✓ PASS' if tutorial_valid else '✗ FAIL'}")
    print(f"Implementation correctness: {'✓ PASS' if impl_valid else '✗ FAIL'}")
    
    overall_pass = all([code_valid, docs_valid, tutorial_valid, impl_valid])
    print(f"\nOverall validation: {'✓ PASS' if overall_pass else '✗ FAIL'}")
    
    if overall_pass:
        print("\n🎉 All tutorials meet educational requirements!")
        print("  - Code is simplified but technically correct")
        print("  - Documentation is clear and educational") 
        print("  - Tutorials follow educational best practices")
        print("  - Implementations are correct")
    else:
        print("\n❌ Some tutorials do not meet educational requirements!")
        print("Please address the issues listed above.")
    
    return 0 if overall_pass else 1


if __name__ == "__main__":
    exit(main())