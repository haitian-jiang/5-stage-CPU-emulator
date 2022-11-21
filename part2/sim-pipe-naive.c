#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/* An implementation of 5-stage classic pipeline simulation */

/* don't count instructions flag, enabled by default, disable for inst count */
#undef NO_INSN_COUNT

#include "host.h"
#include "misc.h"
#include "machine.h"
#include "regs.h"
#include "memory.h"
#include "loader.h"
#include "syscall.h"
#include "dlite.h"
#include "sim.h"
#include "sim-pipe.h"

/* simulated registers */
static struct regs_t regs;

/* simulated memory */
static struct mem_t *mem = NULL;

/* register simulator-specific options */
void
sim_reg_options(struct opt_odb_t *odb)
{
  opt_reg_header(odb, 
"sim-pipe: This simulator implements based on sim-fast.\n"
         );
}

/* check simulator-specific option values */
void
sim_check_options(struct opt_odb_t *odb, int argc, char **argv)
{
  if (dlite_active)
    fatal("sim-pipe does not support DLite debugging");
}

/* register simulator-specific statistics */
void
sim_reg_stats(struct stat_sdb_t *sdb)
{
#ifndef NO_INSN_COUNT
  stat_reg_counter(sdb, "sim_num_insn",
           "total number of instructions executed",
           &sim_num_insn, sim_num_insn, NULL);
#endif /* !NO_INSN_COUNT */
  stat_reg_int(sdb, "sim_elapsed_time",
           "total simulation time in seconds",
           &sim_elapsed_time, 0, NULL);
#ifndef NO_INSN_COUNT
  stat_reg_formula(sdb, "sim_inst_rate",
           "simulation speed (in insts/sec)",
           "sim_num_insn / sim_elapsed_time", NULL);
#endif /* !NO_INSN_COUNT */
  ld_reg_stats(sdb);
  mem_reg_stats(mem, sdb);
}


struct ifid_buf fd;
struct idex_buf de;
struct exmem_buf em;
struct memwb_buf mw;
struct wb_buf wb;

#define DNA         (-1)

/* general register dependence decoders */
#define DGPR(N)         (N)
#define DGPR_D(N)       ((N) &~1)

/* floating point register dependence decoders */
#define DFPR_L(N)       (((N)+32)&~1)
#define DFPR_F(N)       (((N)+32)&~1)
#define DFPR_D(N)       (((N)+32)&~1)

/* miscellaneous register dependence decoders */
#define DHI         (0+32+32)
#define DLO         (1+32+32)
#define DFCC        (2+32+32)
#define DTMP        (3+32+32)

/* initialize the simulator */
void
sim_init(void)
{
  /* allocate and initialize register file */
  regs_init(&regs);

  /* allocate and initialize memory space */
  mem = mem_create("mem");
  mem_init(mem);

  /* initialize stage latches*/
 
  /* IF/ID */
  fd.inst.a = NOP;
  fd.inst.b = 0;
  fd.pc = 0;
  /* ID/EX */
  de.inst.a = NOP;
  de.inst.b = 0;
  de.pc = 0;
  de.opcode = DNA;
  de.rs = DNA;
  de.rd = DNA;
  de.rt = DNA;
  de.rs_val = 0;
  de.rt_val = 0;
  /* EX/MEM */
  em.inst.a = NOP;
  em.inst.b = 0;
  em.pc = 0;
  em.load_mem = 0;
  em.write_rt = 0;
  em.write_rd = 0;
  em.write_enable = 0;
  em.ALUout = 0;
  em.data_in = 0;
  em.rd = DNA;
  em.rt = DNA;
  /* MEM/WB */
  mw.inst.a = NOP;
  mw.inst.b = 0;
  mw.pc = 0;
  mw.write_rt = 0;
  mw.write_rd = 0;
  mw.load_mem = 0;
  mw.from_mem = 0;
  mw.ALUout = 0;
  mw.rt = DNA;
  mw.rd = DNA;
}

/* load program into simulated state */
void
sim_load_prog(char *fname,      /* program to load */
          int argc, char **argv,    /* program arguments */
          char **envp)      /* program environment */
{
  /* load program text and data, set up environment, memory, and regs */
  ld_load_prog(fname, argc, argv, envp, &regs, mem, TRUE);
}

/* print simulator-specific configuration information */
void
sim_aux_config(FILE *stream)
{  
    /* nothing currently */
}

/* dump simulator-specific auxiliary simulator statistics */
void
sim_aux_stats(FILE *stream)
{  /* nada */}

/* un-initialize simulator-specific state */
void 
sim_uninit(void)
{ /* nada */ }


/*
 * configure the execution engine
 */

/* next program counter */
#define SET_NPC(EXPR)       (regs.regs_NPC = (EXPR))

/* current program counter */
#define CPC         (regs.regs_PC)

/* general purpose registers */
#define GPR(N)          (regs.regs_R[N])
#define SET_GPR(N,EXPR)     (regs.regs_R[N] = (EXPR))
#define DECLARE_FAULT(EXP)  {;}
#if defined(TARGET_PISA)

/* floating point registers, L->word, F->single-prec, D->double-prec */
#define FPR_L(N)        (regs.regs_F.l[(N)])
#define SET_FPR_L(N,EXPR)   (regs.regs_F.l[(N)] = (EXPR))
#define FPR_F(N)        (regs.regs_F.f[(N)])
#define SET_FPR_F(N,EXPR)   (regs.regs_F.f[(N)] = (EXPR))
#define FPR_D(N)        (regs.regs_F.d[(N) >> 1])
#define SET_FPR_D(N,EXPR)   (regs.regs_F.d[(N) >> 1] = (EXPR))

/* miscellaneous register accessors */
#define SET_HI(EXPR)        (regs.regs_C.hi = (EXPR))
#define HI          (regs.regs_C.hi)
#define SET_LO(EXPR)        (regs.regs_C.lo = (EXPR))
#define LO          (regs.regs_C.lo)
#define FCC         (regs.regs_C.fcc)
#define SET_FCC(EXPR)       (regs.regs_C.fcc = (EXPR))

#endif

/* precise architected memory state accessor macros */
#define READ_BYTE(SRC, FAULT)                       \
  ((FAULT) = md_fault_none, MEM_READ_BYTE(mem, (SRC)))
#define READ_HALF(SRC, FAULT)                       \
  ((FAULT) = md_fault_none, MEM_READ_HALF(mem, (SRC)))
#define READ_WORD(SRC, FAULT)                       \
  ((FAULT) = md_fault_none, MEM_READ_WORD(mem, (SRC)))
#ifdef HOST_HAS_QWORD
#define READ_QWORD(SRC, FAULT)                      \
  ((FAULT) = md_fault_none, MEM_READ_QWORD(mem, (SRC)))
#endif /* HOST_HAS_QWORD */

#define WRITE_BYTE(SRC, DST, FAULT)                 \
  ((FAULT) = md_fault_none, MEM_WRITE_BYTE(mem, (DST), (SRC)))
#define WRITE_HALF(SRC, DST, FAULT)                 \
  ((FAULT) = md_fault_none, MEM_WRITE_HALF(mem, (DST), (SRC)))
#define WRITE_WORD(SRC, DST, FAULT)                 \
  ((FAULT) = md_fault_none, MEM_WRITE_WORD(mem, (DST), (SRC)))
#ifdef HOST_HAS_QWORD
#define WRITE_QWORD(SRC, DST, FAULT)                    \
  ((FAULT) = md_fault_none, MEM_WRITE_QWORD(mem, (DST), (SRC)))
#endif /* HOST_HAS_QWORD */

/* system call handler macro */
#define SYSCALL(INST)   sys_syscall(&regs, mem_access, mem, INST, TRUE)

#ifndef NO_INSN_COUNT
#define INC_INSN_CTR()  sim_num_insn++
#else /* !NO_INSN_COUNT */
#define INC_INSN_CTR()  /* nada */
#endif /* NO_INSN_COUNT */


/* start simulation, program loaded, processor precise state initialized */
void
sim_main(void)
{
  fprintf(stderr, "sim: ** starting *pipe* functional simulation **\n");

  /* must have natural byte/word ordering */
  if (sim_swap_bytes || sim_swap_words)
    fatal("sim: *pipe* functional simulation cannot swap bytes or words");

  /* set up initial default next PC */
  regs.regs_NPC = regs.regs_PC;
  /* maintain $r0 semantics */
  regs.regs_R[MD_REG_ZERO] = 0;
 
  while (TRUE)
  {
      /*start your pipeline simulation here*/
    INC_INSN_CTR();
    // check_and_set_bubble();
    do_wb();
    do_mem();
    do_ex();
    do_id();
    do_if();    
    print_trace();
  }
}

void do_if(){
    CPC = regs.regs_NPC;
    md_inst_t inst;
    MD_FETCH_INSTI(inst, mem, CPC);
    SET_NPC(CPC + sizeof(md_inst_t));

    fd.inst = inst;
    fd.pc = CPC;
}

void do_id(){
    de.inst = fd.inst;
    de.pc = fd.pc;
    md_inst_t inst = fd.inst;
    SET_OPCODE(de.opcode, inst);
    de.rd = RD;
    de.rs = RS;
    de.rt = RT;
    de.rs_val = GPR(RS);
    de.rt_val = GPR(RT);
}

void do_ex(){
    em.inst = de.inst;
    em.pc = de.pc;
    md_inst_t inst = em.inst;
    em.rt = de.rt;
    em.rd = de.rd;

    em.load_mem = 0;
    em.write_rt = 0;
    em.write_rd = 0;
    em.write_enable = 0;

    switch (de.opcode){
        case ADD:
            em.ALUout = (int)de.rs_val + (int)de.rt_val;
            em.write_rd = 1;
            break;
        case ADDU:
            em.ALUout = (unsigned)de.rs_val + (unsigned)de.rt_val;
            em.write_rd = 1;
            break;
        case SUBU:
            em.ALUout = (unsigned)de.rs_val - (unsigned)de.rt_val;
            em.write_rd = 1;
            break;
        case ADDIU:
            em.ALUout = (unsigned)de.rs_val + IMM;
            em.write_rt = 1;
            break;
        case ANDI:
            em.ALUout = de.rs_val & UIMM;
            em.write_rt = 1;
            break;
        case LUI:
            em.ALUout = UIMM << 16;
            em.write_rt = 1;
            break;
        case LW:
            em.ALUout = de.rs_val + OFS;
            em.write_rt = 1;
            em.load_mem = 1;
            break;
        case SLL:
            em.ALUout = de.rt_val << SHAMT;
            em.write_rd = 1;
            break;
        case SW:
            em.ALUout = de.rs_val + OFS;
            em.data_in = de.rt_val;
            em.write_enable = 1;
            break;
        case SLTI:
            em.ALUout = de.rs_val < IMM;
            em.write_rt = 1;
            break;
        case JUMP:
            break;
    }
}

void do_mem(){
    mw.inst = em.inst;
    mw.pc = em.pc;
    mw.ALUout = em.ALUout;
    mw.write_rt = em.write_rt;
    mw.write_rd = em.write_rd;
    mw.load_mem = em.load_mem;
    mw.rt = em.rt;
    mw.rd = em.rd;
    
    if (em.load_mem){
        mw.from_mem = MEM_READ_WORD(mem, em.ALUout);
    }
    if (em.write_enable){
        MEM_WRITE_WORD(mem, em.ALUout, em.data_in);
    }
}                                                                                        

void do_wb(){
    wb.inst = mw.inst;
    wb.pc = mw.pc;
    md_inst_t inst = wb.inst;

    int opcode;
    SET_OPCODE(opcode, wb.inst);
    if(opcode == SYSCALL){
        // printf("Loop terminated. Result = %d\n",GPR(6));
        SET_GPR(2, SS_SYS_exit);
        SYSCALL(mw.inst);
    }

    if (mw.load_mem){
        if (mw.write_rt){
            SET_GPR(RT, mw.from_mem);
        }if (mw.write_rd){
            SET_GPR(RD, mw.from_mem);
        }
    }else{
        if (mw.write_rt){
            SET_GPR(RT, mw.ALUout);
        }if (mw.write_rd){
            SET_GPR(RD, mw.ALUout);
        }
    }
}

void print_trace()
{
    printf("[Cycle %3d]---------------------------------\n", sim_num_insn);
    printf("[IF]\t");md_print_insn(fd.inst, fd.pc, stdout);printf("\n");
    printf("[ID]\t");md_print_insn(de.inst, de.pc, stdout);printf("\n");
    printf("[EX]\t");md_print_insn(em.inst, em.pc, stdout);printf("\n");
    printf("[MEM]\t");md_print_insn(mw.inst, mw.pc, stdout);printf("\n");
    printf("[WB]\t");md_print_insn(wb.inst, wb.pc, stdout);printf("\n");
    printf("[REGS]r2=%d r3=%d r4=%d r5=%d r6=%d mem = %d\n", 
            GPR(2),GPR(3),GPR(4),GPR(5),GPR(6),MEM_READ_WORD(mem, GPR(30)+16));
    printf("--------------------------------------------\n\n");
}
