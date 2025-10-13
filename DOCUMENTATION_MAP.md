# Documentation Cross-Reference Map

**Last Updated:** October 13, 2025  
**Branch:** mpc-dev-stageC  
**Purpose:** Navigation guide for all Stage C method documentation

---

## 📚 Document Hierarchy

```
projectDiagnosis.md (Main System Documentation)
    │
    ├─→ STAGE_C_METHODS_COMPLETE_ANALYSIS.md
    │   └─ Complete comparison of all methods (0, 1, 4, MPC)
    │   └─ Decision tree for method selection
    │   └─ Usage guide and recommendations
    │
    ├─→ MPC_IMPLEMENTATION_ANALYSIS.md
    │   └─ Deep dive into Methods 2 & 3 proposals
    │   └─ True MPC vs proposed methods comparison
    │   └─ Implementation roadmap and decision tree
    │
    ├─→ g5wbcMpcDesign.md
    │   └─ Mathematical formulation of true NMPC
    │   └─ Nonholonomic dynamics integration
    │   └─ Collision avoidance strategy
    │
    └─→ docs/METHOD_NUMBERING_GUIDE.md
        └─ Complete reference for Methods 0, 1, 4
        └─ Code examples and usage patterns
        └─ Performance comparison tables
```

---

## 🎯 Quick Navigation

### Looking for...

**"What methods are available?"**
→ [docs/METHOD_NUMBERING_GUIDE.md](docs/METHOD_NUMBERING_GUIDE.md)

**"How do all methods compare?"**
→ [STAGE_C_METHODS_COMPLETE_ANALYSIS.md](STAGE_C_METHODS_COMPLETE_ANALYSIS.md)

**"What are Methods 2 and 3 in projectDiagnosis.md?"**
→ [MPC_IMPLEMENTATION_ANALYSIS.md](MPC_IMPLEMENTATION_ANALYSIS.md)

**"How to implement true MPC?"**
→ [g5wbcMpcDesign.md](g5wbcMpcDesign.md) + [MPC_IMPLEMENTATION_ANALYSIS.md](MPC_IMPLEMENTATION_ANALYSIS.md)

**"Which method should I use?"**
→ [STAGE_C_METHODS_COMPLETE_ANALYSIS.md](STAGE_C_METHODS_COMPLETE_ANALYSIS.md) - See decision tree

**"How does the system work overall?"**
→ [projectDiagnosis.md](projectDiagnosis.md) - Main documentation

---

## 📍 Cross-References in projectDiagnosis.md

### Header Section (Lines 1-30)
- ✅ Links to all analysis documents
- ✅ October 2025 update summary

### Section 10: Methods 2 & 3 Discussion
- **Line ~4402:** Method 2 (Iterative Feedback)
  - ✅ Links to [MPC_IMPLEMENTATION_ANALYSIS.md](MPC_IMPLEMENTATION_ANALYSIS.md)
  
- **Line ~4595:** Method 3 (Differential IK QP)
  - ✅ Links to [MPC_IMPLEMENTATION_ANALYSIS.md](MPC_IMPLEMENTATION_ANALYSIS.md)
  - ✅ Links to [g5wbcMpcDesign.md](g5wbcMpcDesign.md)

---

## 📊 Method Implementation Status

| Method | Document | Status | Link |
|--------|----------|--------|------|
| **0** (pureIk) | METHOD_NUMBERING_GUIDE.md | ✅ Implemented | [Guide](docs/METHOD_NUMBERING_GUIDE.md#method-0-pure-ik-pureik) |
| **1** (ppForIk) | METHOD_NUMBERING_GUIDE.md | ✅ Production | [Guide](docs/METHOD_NUMBERING_GUIDE.md#method-1-pp-for-ik-ppforik---current-default) |
| **2** (Iterative) | MPC_IMPLEMENTATION_ANALYSIS.md | 💡 Proposed | [Analysis](MPC_IMPLEMENTATION_ANALYSIS.md#method-2-iterative-feedback-from-projectdiagnosismd) |
| **3** (Diff IK QP) | MPC_IMPLEMENTATION_ANALYSIS.md | ⏳ Deferred | [Analysis](MPC_IMPLEMENTATION_ANALYSIS.md#method-3-differential-ik-with-qp-from-projectdiagnosismd) |
| **4** (ppFirst) | METHOD_NUMBERING_GUIDE.md | ✅ Implemented | [Guide](docs/METHOD_NUMBERING_GUIDE.md#method-4-pp-first-with-gik-refinement-ppfirst-) |
| **MPC** (True NMPC) | g5wbcMpcDesign.md | 📋 Design | [Design](g5wbcMpcDesign.md) |

---

## 🔄 Document Update History

### October 13, 2025
- ✅ Added cross-references in projectDiagnosis.md header
- ✅ Added links in Method 2 section (line ~4402)
- ✅ Added links in Method 3 section (line ~4595)
- ✅ Created STAGE_C_METHODS_COMPLETE_ANALYSIS.md
- ✅ Created MPC_IMPLEMENTATION_ANALYSIS.md
- ✅ Merged Method 4 implementation from origin/main
- ✅ Added g5wbcMpcDesign.md for MPC formulation

### October 12, 2025
- Original projectDiagnosis.md created
- Methods 2 & 3 proposed in Section 10

---

## 💡 Tips

1. **Start with projectDiagnosis.md** - It's the main system documentation
2. **Use METHOD_NUMBERING_GUIDE.md** - For quick method reference
3. **Read STAGE_C_METHODS_COMPLETE_ANALYSIS.md** - For decision making
4. **Consult MPC_IMPLEMENTATION_ANALYSIS.md** - For implementation planning
5. **Reference g5wbcMpcDesign.md** - For mathematical details

---

## 🎯 Common Workflows

### Workflow 1: Choosing a Method
```
1. Read: STAGE_C_METHODS_COMPLETE_ANALYSIS.md (Decision Tree)
2. Reference: METHOD_NUMBERING_GUIDE.md (Method details)
3. Implement: Follow code examples in guide
```

### Workflow 2: Understanding Proposals
```
1. Read: projectDiagnosis.md Section 10 (Methods 2 & 3)
2. Deep dive: MPC_IMPLEMENTATION_ANALYSIS.md
3. Compare: STAGE_C_METHODS_COMPLETE_ANALYSIS.md
```

### Workflow 3: Implementing MPC
```
1. Read: g5wbcMpcDesign.md (Mathematical formulation)
2. Plan: MPC_IMPLEMENTATION_ANALYSIS.md (Implementation roadmap)
3. Test: Follow Phase 1-3 in implementation plan
```

---

**Status:** All cross-references complete ✅  
**Ready for:** Implementation decisions
