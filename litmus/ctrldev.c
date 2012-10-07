#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/vmalloc.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>

#include <litmus/litmus.h>

/* only one page for now, but we might want to add a RO version at some point */

#define CTRL_NAME        "litmus/ctrl"

static struct workqueue_struct *wq_litmus_dealloc;

struct litmus_dealloc_work {
	struct work_struct work_struct;
	void *ctrl_page_mem;
#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
	void *ctrl_page_unmap;
#endif
};

static void litmus_dealloc(struct work_struct *work_in)
{
	struct litmus_dealloc_work *work = container_of(work_in,
			struct litmus_dealloc_work, work_struct);
#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
	TRACE("vunmap() control page %p.\n", work->ctrl_page_unmap);
	vunmap(work->ctrl_page_unmap);
#endif
	TRACE("freeing ctrl_page %p\n", work->ctrl_page_mem);
	free_page((unsigned long) work->ctrl_page_mem);

	kfree((void*) work);
}

void litmus_schedule_deallocation(struct task_struct *t)
{
	struct litmus_dealloc_work *work;

	if (NULL == tsk_rt(t)->ctrl_page)
		return;

	work = kmalloc(sizeof(*work), GFP_ATOMIC);
	if (!work) {
		WARN(1, "Could not allocate LITMUS deallocation work.\n");
		return;
	}

	INIT_WORK(&work->work_struct, litmus_dealloc);

#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
	work->ctrl_page_mem = tsk_rt(t)->ctrl_page_orig;
	work->ctrl_page_unmap = tsk_rt(t)->ctrl_page;
#else
	work->ctrl_page_mem = tsk_rt(t)->ctrl_page;
#endif
	queue_work(wq_litmus_dealloc, &work->work_struct);
}

#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
/*
 * remap_noncached - creates a non-cached memory "shadow mapping"
 * @addr:	memory base virtual address
 * @len:	length to remap
 *
 * The caller should vunmap(addr) when the mapping is no longer needed.
 * The caller should also save the original @addr to free it later.
 */
static void * remap_noncached(void *addr, size_t len)
{
	void *vaddr;
	int nr_pages = DIV_ROUND_UP(offset_in_page(addr) + len, PAGE_SIZE);
	struct page **pages = kmalloc(nr_pages * sizeof(*pages), GFP_KERNEL);
	void *page_addr = (void *)((unsigned long)addr & PAGE_MASK);
	int i;

	if (NULL == pages) {
		TRACE_CUR("No memory!\n");
		return ERR_PTR(-ENOMEM);
	}

	for (i = 0; i < nr_pages; i++) {
		if (is_vmalloc_or_module_addr(page_addr)) {
			kfree(pages);
			TRACE_CUR("Remapping vmalloc or module memory?\n");
			return ERR_PTR(-EINVAL);
		}

		pages[i] = virt_to_page(page_addr);
		if (NULL == pages[i]) {
			kfree(pages);
			TRACE_CUR("Bad virtual address.\n");
			return ERR_PTR(-EINVAL);
		}
		page_addr += PAGE_SIZE;
	}

	vaddr = vmap(pages, nr_pages, VM_MAP, pgprot_noncached(PAGE_KERNEL));
	kfree(pages);
	if (NULL == vaddr) {
		TRACE_CUR("vmap() failed.\n");
		return ERR_PTR(-ENOMEM);
	}
	return vaddr + offset_in_page(addr);
}
#endif

/* allocate t->rt_param.ctrl_page*/
static int alloc_ctrl_page(struct task_struct *t)
{
	void *mem;
#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
	void *mem_remap;
#endif
	int err = 0;

	/* only allocate if the task doesn't have one yet */
	if (!tsk_rt(t)->ctrl_page) {
		mem = (void*) get_zeroed_page(GFP_KERNEL);
		if (!mem) {
			err = -ENOMEM;
			goto out;
		}

#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
		mem_remap = remap_noncached(mem, PAGE_SIZE);
		if (IS_ERR(mem_remap)) {
			err = PTR_ERR(mem_remap);
			free_page((unsigned long) mem);
			goto out;
		}
		tsk_rt(t)->ctrl_page_orig = mem;
		tsk_rt(t)->ctrl_page = mem_remap;
		TRACE_TASK(t, "ctrl_page_orig = %p\n",
				tsk_rt(t)->ctrl_page_orig);
#else
		tsk_rt(t)->ctrl_page = mem;
#endif

		/* will get de-allocated in task teardown */
		TRACE_TASK(t, "%s ctrl_page = %p\n", __FUNCTION__,
			   tsk_rt(t)->ctrl_page);
	}
out:
	return err;
}

static int map_ctrl_page(struct task_struct *t, struct vm_area_struct* vma)
{
	struct page *ctrl;
	int err;

#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
	/* vm_insert_page() using the "real" vaddr, not the shadow mapping. */
	ctrl = virt_to_page(tsk_rt(t)->ctrl_page_orig);
#else
	ctrl = virt_to_page(tsk_rt(t)->ctrl_page);
#endif

	TRACE_CUR(CTRL_NAME
		  ": mapping %p (pfn:%lx) to 0x%lx (prot:%lx)\n",
		  tsk_rt(t)->ctrl_page, page_to_pfn(ctrl), vma->vm_start,
		  vma->vm_page_prot);

	/* Map it into the vma. */
	err = vm_insert_page(vma, vma->vm_start, ctrl);

	if (err)
		TRACE_CUR(CTRL_NAME ": vm_insert_page() failed (%d)\n", err);

	return err;
}

static void litmus_ctrl_vm_close(struct vm_area_struct* vma)
{
	TRACE_CUR("%s flags=0x%x prot=0x%x\n", __FUNCTION__,
		  vma->vm_flags, vma->vm_page_prot);

	TRACE_CUR(CTRL_NAME
		  ": %p:%p vma:%p vma->vm_private_data:%p closed.\n",
		  (void*) vma->vm_start, (void*) vma->vm_end, vma,
		  vma->vm_private_data);
}

static int litmus_ctrl_vm_fault(struct vm_area_struct* vma,
				      struct vm_fault* vmf)
{
	TRACE_CUR("%s flags=0x%x (off:%ld)\n", __FUNCTION__,
		  vma->vm_flags, vmf->pgoff);

	/* This function should never be called, since all pages should have
	 * been mapped by mmap() already. */
	WARN_ONCE(1, "Page faults should be impossible in the control page\n");

	return VM_FAULT_SIGBUS;
}

static struct vm_operations_struct litmus_ctrl_vm_ops = {
	.close = litmus_ctrl_vm_close,
	.fault = litmus_ctrl_vm_fault,
};

static int litmus_ctrl_mmap(struct file* filp, struct vm_area_struct* vma)
{
	int err = 0;

	/* first make sure mapper knows what he's doing */

	/* you can only get one page */
	if (vma->vm_end - vma->vm_start != PAGE_SIZE)
		return -EINVAL;

	/* you can only map the "first" page */
	if (vma->vm_pgoff != 0)
		return -EINVAL;

	/* you can't share it with anyone */
	if (vma->vm_flags & (VM_MAYSHARE | VM_SHARED))
		return -EINVAL;

	vma->vm_ops = &litmus_ctrl_vm_ops;
	/* This mapping should not be kept across forks,
	 * cannot be expanded, and is not a "normal" page. */
	vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND | VM_IO;

	/* We don't want the first write access to trigger a "minor" page fault
	 * to mark the page as dirty.  This is transient, private memory, we
	 * don't care if it was touched or not. __S011 means RW access, but not
	 * execute, and avoids copy-on-write behavior.
	 * See protection_map in mmap.c.  */
#ifdef CONFIG_ARCH_NEEDS_UNCACHED_CONTROL_PAGE
	vma->vm_page_prot = pgprot_noncached(__S011);
#else
	vma->vm_page_prot = __S011;
#endif

	err = alloc_ctrl_page(current);
	if (!err)
		err = map_ctrl_page(current, vma);

	TRACE_CUR("%s flags=0x%x prot=0x%lx\n",
		  __FUNCTION__, vma->vm_flags, vma->vm_page_prot);

	return err;
}

static struct file_operations litmus_ctrl_fops = {
	.owner = THIS_MODULE,
	.mmap  = litmus_ctrl_mmap,
};

static struct miscdevice litmus_ctrl_dev = {
	.name  = CTRL_NAME,
	.minor = MISC_DYNAMIC_MINOR,
	.fops  = &litmus_ctrl_fops,
};

static int __init init_litmus_ctrl_dev(void)
{
	int err;

	BUILD_BUG_ON(sizeof(struct control_page) > PAGE_SIZE);

	printk("Initializing LITMUS^RT control device.\n");
	err = misc_register(&litmus_ctrl_dev);
	if (err)
		printk("Could not allocate %s device (%d).\n", CTRL_NAME, err);

	wq_litmus_dealloc = alloc_workqueue("litmus_dealloc",
			WQ_NON_REENTRANT|WQ_MEM_RECLAIM, 0);
	if (NULL == wq_litmus_dealloc) {
		printk("Could not allocate vunmap workqueue.\n");
		misc_deregister(&litmus_ctrl_dev);
	}
	return err;
}

static void __exit exit_litmus_ctrl_dev(void)
{
	flush_workqueue(wq_litmus_dealloc);
	destroy_workqueue(wq_litmus_dealloc);
	misc_deregister(&litmus_ctrl_dev);
}

module_init(init_litmus_ctrl_dev);
module_exit(exit_litmus_ctrl_dev);
