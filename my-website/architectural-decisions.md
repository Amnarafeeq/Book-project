# Architectural Decisions for Physical AI & Humanoid Robotics Docusaurus Site

## 1. Technology Stack Decision

**Decision**: Use Docusaurus as the static site generator for the textbook website.

**Rationale**:
- Docusaurus provides excellent documentation site features out of the box
- Strong support for technical content with code blocks and syntax highlighting
- Built-in search, versioning, and internationalization capabilities
- Active community and regular updates
- Good integration with GitHub for content management

## 2. Theme and Styling Architecture

**Decision**: Implement a custom theme with purple (#A78BFA) as primary color, black headers/footers, and white/soft gray content areas.

**Rationale**:
- Purple color scheme aligns with the futuristic, high-tech theme of robotics and AI
- Black headers/footers provide a professional, high-tech aesthetic
- White/soft gray backgrounds ensure readability of technical content
- Dark mode support accommodates different user preferences and use cases

## 3. Component Architecture

**Decision**: Create custom React components for specialized content display (ModuleCard, CalloutBox, CustomNavbar, CustomFooter).

**Rationale**:
- Reusable components ensure consistency across the textbook
- Custom components allow for specialized functionality like themed callouts
- Modular approach makes maintenance easier
- Components can be extended with additional props as needed

## 4. Navigation Structure

**Decision**: Organize content in a hierarchical structure with modules as the main navigation category.

**Rationale**:
- Modules represent the core learning progression of the textbook
- Hierarchical structure makes it easy to find specific content
- Sidebar navigation allows for easy switching between modules
- Clear separation between different types of content (modules, hardware, capstone, etc.)

## 5. Content Organization

**Decision**: Structure content as a sequence of 4 main modules plus supporting pages (overview, hardware, capstone, glossary, resources).

**Rationale**:
- Sequential modules allow for progressive learning
- Supporting pages provide necessary context and reference material
- Each module follows a consistent structure (introduction, skills, tools, examples)
- Clear learning path from basic concepts to advanced integration

## 6. Typography and Visual Design

**Decision**: Use Inter for body text and JetBrains Mono for code, with subtle neon glow effects for important elements.

**Rationale**:
- Inter is highly readable for technical documentation
- JetBrains Mono is optimized for code display and technical content
- Neon glow effects reinforce the high-tech, AI/robotics theme
- Subtle effects enhance without overwhelming the content

## 7. Responsive Design Approach

**Decision**: Implement responsive design that works across mobile, tablet, and desktop devices.

**Rationale**:
- Students may access content on various devices
- Mobile-friendly design accommodates on-the-go learning
- Responsive components ensure consistent experience across devices
- Critical for accessibility and broad adoption

## 8. Performance Optimization

**Decision**: Optimize for fast loading times and efficient resource usage.

**Rationale**:
- Fast loading times improve user experience and SEO
- Technical students expect high-performance applications
- Optimized assets reduce bandwidth usage
- Better performance across different network conditions

## 9. Accessibility Implementation

**Decision**: Implement WCAG 2.1 AA compliance standards throughout the site.

**Rationale**:
- Ensures the textbook is accessible to all learners
- Required for educational institutions
- Good accessibility practices benefit all users
- Legal compliance in many jurisdictions

## 10. Code Example Integration

**Decision**: Integrate syntax-highlighted code examples directly in the content with custom styling.

**Rationale**:
- Critical for a technical textbook to have readable code examples
- Custom styling maintains visual consistency with the theme
- Syntax highlighting improves comprehension
- Inline examples connect theory with practice